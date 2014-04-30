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

#include "SurgSim/Devices/DeviceFilters/PoseIntegrator.h"

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Device
{

PoseIntegrator::PoseIntegrator(const std::string& name) :
	CommonDevice(name),
	m_poseResult(PoseType::Identity()),
	m_rate(1.0)
{
}

const PoseIntegrator::PoseType& PoseIntegrator::integrate(const PoseType& pose)
{
	// Note: we apply translation and rotation separately.  This is NOT the same as (m_poseResult * pose)!
	m_poseResult.pretranslate(pose.translation());
	m_poseResult.rotate(pose.rotation());
	return m_poseResult;
}

bool PoseIntegrator::initialize()
{
	return true;
}

bool PoseIntegrator::finalize()
{
	return true;
}

bool PoseIntegrator::isInitialized() const
{
	return true;
}

void PoseIntegrator::initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	getInitialInputData() = inputData;
	getInputData() = inputData;

	SurgSim::Math::RigidTransform3d pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		m_poseResult = pose;
	}
}

void PoseIntegrator::handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	getInputData() = inputData;
	SurgSim::Math::RigidTransform3d pose;
	if (inputData.poses().get(SurgSim::DataStructures::Names::POSE, &pose))
	{
		// Before updating the current pose, use it to calculate the angular velocity.
		Vector3d unused;
		if (inputData.vectors().get(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, &unused))
		{
			Vector3d rotationAxis;
			double angle;
			SurgSim::Math::computeAngleAndAxis(pose.rotation(), &angle, &rotationAxis);
			rotationAxis = m_poseResult.rotation() * rotationAxis; // rotate the axis into global space
			getInputData().vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, rotationAxis *
				angle * m_rate);
		}

		getInputData().poses().set(SurgSim::DataStructures::Names::POSE, integrate(pose));

		if (inputData.vectors().get(SurgSim::DataStructures::Names::LINEAR_VELOCITY, &unused))
		{
			getInputData().vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, pose.translation() * m_rate);
		}
	}
	pushInput();
}

bool PoseIntegrator::requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData)
{
	bool state = pullOutput();
	if (state)
	{
		*outputData = getOutputData();
	}
	return state;
}

void PoseIntegrator::setRate(double rate)
{
	m_rate = rate;
}

double PoseIntegrator::getRate() const
{
	return m_rate;
}


};  // namespace Device
};  // namespace SurgSim
