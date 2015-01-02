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
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> AngularMeasurementMatrix;
typedef Eigen::Matrix<double, 3, 1> AngularMeasurementVector;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> AngularObservationMatrix;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AngularStateMatrix;
typedef Eigen::Matrix<double, 6, 1> AngularStateVector;

/// Measure translational velocity by measuring translation, a 3-vector. The state is {position, velocity}.
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> LinearMeasurementMatrix;
typedef Eigen::Matrix<double, 3, 1> LinearMeasurementVector;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> LinearObservationMatrix;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> LinearStateMatrix;
typedef Eigen::Matrix<double, 6, 1> LinearStateVector;

VelocityFromPose::VelocityFromPose(const std::string& name) :
	CommonDevice(name),
	m_firstInput(true),
	m_lastPose(SurgSim::Math::RigidTransform3d::Identity())
{
	m_timer.setMaxNumberOfFrames(1);

	LinearObservationMatrix linearObservationMatrix;
	linearObservationMatrix << LinearMeasurementMatrix::Identity(), LinearMeasurementMatrix::Zero();
	m_linearFilter.setObservationMatrix(linearObservationMatrix);
	m_linearFilter.setInitialState(LinearStateVector::Zero());
	m_linearFilter.setInitialStateCovariance(LinearStateMatrix::Ones() * 1000.0);
	LinearStateMatrix linearProcessNoise;
	linearProcessNoise << LinearMeasurementMatrix::Identity() * 0.1, LinearMeasurementMatrix::Zero(),
							LinearMeasurementMatrix::Zero(), LinearMeasurementMatrix::Identity() * 100.0;
	m_linearFilter.setProcessNoiseCovariance(linearProcessNoise);//LinearStateMatrix::Identity() * 1.0);
	m_linearFilter.setMeasurementNoiseCovariance(LinearMeasurementMatrix::Identity() * 1.0);

	AngularObservationMatrix angularObservationMatrix;
	angularObservationMatrix << AngularMeasurementMatrix::Identity(), AngularMeasurementMatrix::Zero();
	m_angularFilter.setObservationMatrix(angularObservationMatrix);
	m_angularFilter.setInitialState(AngularStateVector::Zero());
	m_angularFilter.setInitialStateCovariance(AngularStateMatrix::Ones() * 1000.0);
	m_angularFilter.setProcessNoiseCovariance(AngularStateMatrix::Identity() * 10.0);
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
	if (m_firstInput)
	{
		m_firstInput = false;

		if (!inputData.vectors().hasEntry(SurgSim::DataStructures::Names::LINEAR_VELOCITY) ||
			!inputData.vectors().hasEntry(SurgSim::DataStructures::Names::ANGULAR_VELOCITY))
		{
			SurgSim::DataStructures::DataGroupBuilder builder;
			builder.addEntriesFrom(inputData);
			builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
			builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
			getInputData() = builder.createData();
			m_copier = std::make_shared<SurgSim::DataStructures::DataGroupCopier>(inputData, getInputData());
		}
	}

	if (m_copier == nullptr)
	{
		getInputData() = inputData;
	}
	else
	{
		m_copier->copy(inputData, &getInputData());
	}

	PoseType pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		m_lastPose = pose;
		LinearStateVector linearState;
		linearState << pose.translation(), LinearMeasurementVector::Zero();
		m_linearFilter.setInitialState(linearState);
	}
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

	PoseType pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		m_timer.markFrame();
		const double period = m_timer.getLastFramePeriod();
		if ((m_timer.getNumberOfClockFails() == 0) && (boost::math::isnormal(period)) && (period > 0.0))
		{
			LinearStateMatrix linearStateTransition;
			linearStateTransition <<
				LinearMeasurementMatrix::Identity(), period * LinearMeasurementMatrix::Identity(),
				LinearMeasurementMatrix::Zero(),     LinearMeasurementMatrix::Identity();

			m_linearFilter.setStateTransition(linearStateTransition);
			const LinearStateVector& linearState = m_linearFilter.update(pose.translation());
 			getInputData().vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, linearState.segment<3>(3));

			AngularStateMatrix angularStateTransition;
			angularStateTransition <<
				AngularMeasurementMatrix::Identity(), period * AngularMeasurementMatrix::Identity(),
				AngularMeasurementMatrix::Zero(),     AngularMeasurementMatrix::Identity();
			m_angularFilter.setStateTransition(angularStateTransition);
			SurgSim::Math::Vector3d rotation;
			SurgSim::Math::computeRotationVector(pose, m_lastPose, &rotation);
			m_lastPose = pose;
			const AngularStateVector& angularState = m_angularFilter.update(rotation / period);
			getInputData().vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, angularState.segment<3>(0));
		}
		else
		{
			m_timer.start();
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Devices/Filters/VelocityFromPose")) <<
				"The Timer used by " << getName() << " failed to get a good value.  No velocity will be provided.";

			getInputData().vectors().reset(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
			getInputData().vectors().reset(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
		}
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
