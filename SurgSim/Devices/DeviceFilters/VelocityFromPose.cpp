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

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector6d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix66d;

namespace SurgSim
{
namespace Device
{

VelocityFromPose::VelocityFromPose(const std::string& name) :
	CommonDevice(name),
	m_firstInput(true),
	m_linearState(Eigen::Matrix<double, 9, 1>::Zero()),
	m_linearCovariance(Eigen::Matrix<double, 9, 9, Eigen::RowMajor>::Ones() * 1000.0)
{
	m_timer.setMaxNumberOfFrames(1);
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
		m_copier->copy();
	}

	PoseType pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
	//	m_state << pose.translation(), Vector3d::Zero(), pose.rotation().eulerAngles(2, 0, 2), Vector3d::Zero();
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
		m_copier->copy();
	}

	PoseType pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		m_timer.markFrame();
		double period = m_timer.getLastFramePeriod();
		if ((m_timer.getNumberOfClockFails() > 0) || (!boost::math::isnormal(period)))
		{
			m_timer.start();
			period = 0.0;
			SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("Devices/Filters/VelocityFromPose")) <<
				"The Timer used by " << getName() <<
				" failed to get a good value.  The calculated velocities will be zero this update.";
		}

		Vector3d linearVelocity;
		Vector3d angularVelocity;
		calculateVelocity(pose, period, &linearVelocity, &angularVelocity);
 		getInputData().vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, linearVelocity);
 		getInputData().vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, angularVelocity);
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

void VelocityFromPose::calculateVelocity(const PoseType& pose, double period, Vector3d* linearVelocity,
										 Vector3d* angularVelocity)
{
	typedef Eigen::Matrix<double, 12, 1> VectorType;
	typedef Eigen::Matrix<double, 12, 12, Eigen::RowMajor> MatrixType;

	std::cout << pose.translation() << "   " << pose.rotation().eulerAngles(2, 0, 2) << std::endl;

	kalman(pose.translation(), period, &m_linearState, &m_linearCovariance);

	std::cout << m_linearState.transpose() << std::endl;
	*linearVelocity = m_linearState.segment<3>(3);

// 	mat = AngleAxisf(ea[0], Vector3f::UnitZ()) * AngleAxisf(ea[1], Vector3f::UnitX()) * AngleAxisf(ea[2], Vector3f::UnitZ());
 	*angularVelocity = Vector3d::Zero();
}

void VelocityFromPose::kalman(const SurgSim::Math::Vector3d& measurement, double period,
							  Eigen::Matrix<double, 9, 1>* state,
							  Eigen::Matrix<double, 9, 9, Eigen::RowMajor>* covariance)
{
	/// Predict step.
	/// a priori state estimate: x_predicted,k,k-1 = F.x_predicted,k-1,k-1 + B.u
	/// a priori estimate covariance: P_k,k-1 = F.P_k-1,k-1.Ft + Q
	/// We have no control, so the control-input model, B, and the control, u, are zeros.
	Eigen::Matrix<double, 9, 9, Eigen::RowMajor> stateTransition;
	stateTransition <<
		Matrix33d::Identity(), period * Matrix33d::Identity(), period * period * 0.5 * Matrix33d::Identity(),
		Matrix33d::Zero(),     Matrix33d::Identity(), period * Matrix33d::Identity(),
		Matrix33d::Zero(),     Matrix33d::Zero(),     Matrix33d::Identity();
	(*state) = stateTransition * (*state);

	const double processNoise = 10.0;
	Eigen::Matrix<double, 9, 9, Eigen::RowMajor> processNoiseCovariance =
		processNoise * Eigen::Matrix<double, 9, 9, Eigen::RowMajor>::Identity();
	*covariance = stateTransition * (*covariance) * stateTransition.transpose() + processNoiseCovariance;

	/// Update step.
	/// measurement residual: y = z - H.x_predicted,k,k-1
	/// innovation covariance: S = H.P_k,k-1.Ht + R
	/// optimal Kalman gain: K = P_k,k-1.Ht.inv(S_k)
	/// a posteriori state estimate: x_predicted,k,k = x_predicted,k_k-1 + K.y
	/// a posteriori estimate covariance: P_k,k = (I - K.H).P_k,k-1
	/// We assume no observation noise, so the observation noise covariance, R, is zero.
	/// We observe the position, but not velocity or acceleration.
	Eigen::Matrix<double, 3, 9, Eigen::RowMajor> observationMatrix;
	observationMatrix << Matrix33d::Identity(), Matrix33d::Zero(), Matrix33d::Zero();
	const Vector3d innovation = measurement - observationMatrix * (*state);
	const double measurementNoise = 1.0;
	const Matrix33d measurementNoiseCovariance = measurementNoise * SurgSim::Math::Matrix33d::Identity();
	const Matrix33d innovationCovariance = observationMatrix * (*covariance) * observationMatrix.transpose() +
		measurementNoiseCovariance;
	const Eigen::Matrix<double, 9, 3, Eigen::RowMajor> gain =
		(*covariance) * observationMatrix.transpose() * innovationCovariance.inverse();
	*state = (*state) + gain * innovation;
	*covariance = (Eigen::Matrix<double, 9, 9, Eigen::RowMajor>::Identity() - gain * observationMatrix) * (*covariance);
}

};  // namespace Device
};  // namespace SurgSim
