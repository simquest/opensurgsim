// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "SurgSim/Devices/DeviceFilters/VelocityFromPose.h"

#include <boost/math/special_functions/fpclassify.hpp>

#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/DataStructures/DataGroupCopier.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Quaternion.h"

namespace SurgSim
{
namespace Device
{

/// Estimate angular velocity by measuring angular velocity, a 3-vector.  The state is {velocity, acceleration}.
typedef SurgSim::Math::Matrix33d AngularMeasurementMatrix;
typedef SurgSim::Math::Vector3d AngularMeasurementVector;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> AngularObservationMatrix;
typedef SurgSim::Math::Matrix66d AngularStateMatrix;
typedef SurgSim::Math::Vector6d AngularStateVector;

/// Measure translational velocity by measuring translation, a 3-vector. The state is {position, velocity}.
typedef SurgSim::Math::Matrix33d LinearMeasurementMatrix;
typedef SurgSim::Math::Vector3d LinearMeasurementVector;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> LinearObservationMatrix;
typedef SurgSim::Math::Matrix66d LinearStateMatrix;
typedef SurgSim::Math::Vector6d LinearStateVector;

VelocityFromPose::VelocityFromPose(const std::string& name) :
	CommonDevice(name),
	m_lastPose(SurgSim::Math::RigidTransform3d::Identity())
{
	LinearObservationMatrix linearObservationMatrix;
	linearObservationMatrix << LinearMeasurementMatrix::Identity(), LinearMeasurementMatrix::Zero();
	m_linearFilter.setObservationMatrix(linearObservationMatrix);
	m_linearFilter.setInitialState(LinearStateVector::Zero());
	m_linearFilter.setInitialStateCovariance(LinearStateMatrix::Ones() * 1000.0);
	m_linearFilter.setMeasurementNoiseCovariance(LinearMeasurementMatrix::Identity() * 0.1);

	AngularObservationMatrix angularObservationMatrix;
	angularObservationMatrix << AngularMeasurementMatrix::Identity(), AngularMeasurementMatrix::Zero();
	m_angularFilter.setObservationMatrix(angularObservationMatrix);
	m_angularFilter.setInitialState(AngularStateVector::Zero());
	m_angularFilter.setInitialStateCovariance(AngularStateMatrix::Ones() * 1000.0);
	m_angularFilter.setMeasurementNoiseCovariance(AngularMeasurementMatrix::Identity());
}

bool VelocityFromPose::initialize()
{
	return true;
}

bool VelocityFromPose::finalize()
{
	return true;
}

void VelocityFromPose::initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	if (!inputData.vectors().hasEntry(SurgSim::DataStructures::Names::LINEAR_VELOCITY) ||
		!inputData.vectors().hasEntry(SurgSim::DataStructures::Names::ANGULAR_VELOCITY))
	{
		SurgSim::DataStructures::DataGroupBuilder builder;
		builder.addEntriesFrom(inputData);
		builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
		builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
		getInputData() = builder.createData();
		m_copier = std::make_shared<SurgSim::DataStructures::DataGroupCopier>(inputData, &getInputData());
		m_copier->copy(inputData, &getInputData());
	}
	else
	{
		getInputData() = inputData;
	}

	SurgSim::Math::RigidTransform3d pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		m_lastPose = pose;
		LinearStateVector linearState;
		linearState << pose.translation(), LinearMeasurementVector::Zero();
		m_linearFilter.setInitialState(linearState);
	}

	m_lastTime = SurgSim::Framework::Clock::now();
}

void VelocityFromPose::handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	if (m_copier == nullptr)
	{
		getInputData() = inputData;
	}
	else
	{
		m_copier->copy(inputData, &getInputData());
	}

	SurgSim::Math::RigidTransform3d pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		const SurgSim::Framework::Clock::time_point now = SurgSim::Framework::Clock::now();
		const boost::chrono::duration<double> duration = now - m_lastTime;
		const double period = duration.count();
		if ((boost::math::isnormal(period)) && (period > 0.0))
		{
			LinearStateMatrix linearStateTransition;
			linearStateTransition <<
				LinearMeasurementMatrix::Identity(), period * LinearMeasurementMatrix::Identity(),
				LinearMeasurementMatrix::Zero(),     LinearMeasurementMatrix::Identity();
			m_linearFilter.setStateTransition(linearStateTransition);

			LinearStateMatrix linearProcessNoise;
			const double linearCovariance = 1e7;
			const LinearMeasurementMatrix offDiagonalLinearCovariance =
				LinearMeasurementMatrix::Identity() * std::pow(period, 3) * 0.5 * linearCovariance;
			linearProcessNoise <<
				LinearMeasurementMatrix::Identity() * std::pow(period, 4) * 0.25 * linearCovariance,
																							offDiagonalLinearCovariance,
				offDiagonalLinearCovariance, LinearMeasurementMatrix::Identity() * period * period * linearCovariance;
			m_linearFilter.setProcessNoiseCovariance(linearProcessNoise);

			const LinearStateVector& linearState = m_linearFilter.update(pose.translation());
			getInputData().vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, linearState.segment<3>(3));

			AngularStateMatrix angularStateTransition;
			angularStateTransition <<
				AngularMeasurementMatrix::Identity(), period * AngularMeasurementMatrix::Identity(),
				AngularMeasurementMatrix::Zero(),     AngularMeasurementMatrix::Identity();
			m_angularFilter.setStateTransition(angularStateTransition);

			AngularStateMatrix angularProcessNoise;
			const double angularCovariance = 1e8;
			const AngularMeasurementMatrix offDiagonalAngularCovariance =
				AngularMeasurementMatrix::Identity() * std::pow(period, 3) * 0.5 * angularCovariance;
			angularProcessNoise <<
				AngularMeasurementMatrix::Identity() * std::pow(period, 4) * 0.25 * angularCovariance,
																						offDiagonalAngularCovariance,
				offDiagonalAngularCovariance,
											AngularMeasurementMatrix::Identity() * period * period * angularCovariance;
			m_angularFilter.setProcessNoiseCovariance(angularProcessNoise);

			SurgSim::Math::Vector3d rotation;
			SurgSim::Math::computeRotationVector(pose, m_lastPose, &rotation);
			const AngularStateVector& angularState = m_angularFilter.update(rotation / period);
			getInputData().vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, angularState.segment<3>(0));
		}
		else
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Devices/Filters/VelocityFromPose")) <<
				"The Timer used by " << getName() << " failed to get a good value.  No velocity will be provided.";

			getInputData().vectors().reset(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
			getInputData().vectors().reset(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
		}

		m_lastTime = now;
		m_lastPose = pose;
	}
	pushInput();
}

bool VelocityFromPose::requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData)
{
	bool state = pullOutput();
	if (state)
	{
		*outputData = getOutputData();
	}
	return state;
}

};  // namespace Device
};  // namespace SurgSim
