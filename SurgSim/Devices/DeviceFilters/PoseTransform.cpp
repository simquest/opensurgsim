// This filrgSim project.
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

#include "SurgSim/Devices/DeviceFilters/PoseTransform.h"

#include "SurgSim/Math/Matrix.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Device
{

PoseTransform::PoseTransform(const std::string& name) :
	CommonDevice(name),
	m_transform(RigidTransform3d::Identity()),
	m_transformInverse(RigidTransform3d::Identity()),
	m_translationScale(1.0)
{
}

PoseTransform::~PoseTransform()
{
	finalize();
}

bool PoseTransform::initialize()
{
	return true;
}

bool PoseTransform::finalize()
{
	return true;
}

void PoseTransform::initializeInput(const std::string& device, const DataGroup& inputData)
{
	getInitialInputData() = inputData;
	getInputData() = getInitialInputData();
}

void PoseTransform::handleInput(const std::string& device, const DataGroup& inputData)
{
	inputFilter(inputData, &getInputData());
	pushInput();
}

bool PoseTransform::requestOutput(const std::string& device, DataGroup* outputData)
{
	bool state = pullOutput();
	if (state)
	{
		outputFilter(getOutputData(), outputData);
	}
	return state;
}

void PoseTransform::inputFilter(const DataGroup& dataToFilter, DataGroup* result)
{
	*result = dataToFilter;  // Pass on all the data entries.

	// If there is a pose, offset and scale it.
	RigidTransform3d pose;
	if (dataToFilter.poses().get("pose", &pose))
	{
		pose.translation() *= m_translationScale;
		pose = m_transform * pose;
		result->poses().set("pose", pose);
	}

	// If there is a linear velocity, scale it to match the scaled translation.
	Vector3d linearVelocity;
	if (dataToFilter.vectors().get("linearVelocity", &linearVelocity))
	{
		linearVelocity *= m_translationScale;
		result->vectors().set("linearVelocity", linearVelocity);
	}
}

void PoseTransform::outputFilter(const DataGroup& dataToFilter, DataGroup* result)
{
	*result = dataToFilter;  // Pass on all the data entries.

	// Since the haptic devices will compare the data in the output DataGroup to a raw input pose, the filter must
	// reverse any transformation it did to any data the haptic devices use.
	SurgSim::Math::RigidTransform3d inputPose;
	if (dataToFilter.poses().get("inputPose", &inputPose))
	{
		inputPose = m_transformInverse * inputPose;
		inputPose.translation() /= m_translationScale;
		result->poses().set("inputPose", inputPose);
	}

	Vector3d inputLinearVelocity;
	if (dataToFilter.vectors().get("inputLinearVelocity", &inputLinearVelocity))
	{
		inputLinearVelocity /= m_translationScale;
		result->vectors().set("linearVelocity", inputLinearVelocity);
	}
}

void PoseTransform::setTranslationScale(double translationScale)
{
	m_translationScale = translationScale;
}

void PoseTransform::setTransform(const RigidTransform3d& transform)
{
	m_transform = transform;
	m_transformInverse = m_transform.inverse();
}

};  // namespace Device
};  // namespace SurgSim
