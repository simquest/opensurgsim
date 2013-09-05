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

#include "SurgSim/Devices/MultiAxis/PoseIntegrator.h"


namespace SurgSim
{
namespace Device
{

PoseIntegrator::PoseIntegrator(const std::string& name) :
	CommonDevice(name),
	m_poseResult(PoseType::Identity())
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
	// The object was created with no initial input data, but now we can set the initial (and current) input data.
	setInputData(inputData);
}

void PoseIntegrator::handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData)
{
	setInputData(inputData);
	SurgSim::Math::RigidTransform3d pose;
	if (inputData.poses().get("pose", &pose))
	{
		getInputData().poses().set("pose", integrate(pose));
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


};  // namespace Device
};  // namespace SurgSim