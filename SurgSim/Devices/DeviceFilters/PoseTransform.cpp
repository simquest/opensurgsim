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

	RigidTransform3d pose; // If there is a pose, scale the translation, then transform the result.
	if (dataToFilter.poses().get("pose", &pose))
	{
		pose.translation() *= m_translationScale;
		pose = m_transform * pose;
		result->poses().set("pose", pose);
	}

	Vector3d linearVelocity; // If there is a linear velocity, scale then rotate it.
	if (dataToFilter.vectors().get("linearVelocity", &linearVelocity))
	{
		linearVelocity *= m_translationScale;
		linearVelocity = m_transform.linear() * linearVelocity;
		result->vectors().set("linearVelocity", linearVelocity);
	}

	Vector3d angularVelocity; // If there is an angular velocity, rotate it.
	if (dataToFilter.vectors().get("angularVelocity", &angularVelocity))
	{
		angularVelocity = m_transform.linear() * angularVelocity;
		result->vectors().set("angularVelocity", angularVelocity);
	}
}

void PoseTransform::outputFilter(const DataGroup& dataToFilter, DataGroup* result)
{
	*result = dataToFilter;  // Pass on all the data entries.

	// Since the haptic devices will compare the data in the output DataGroup to a raw input pose, the filter must
	// perform the reverse transform and scaling to data used by the haptic devices.
	RigidTransform3d inputPose;
	if (dataToFilter.poses().get("inputPose", &inputPose))
	{
		inputPose = m_transformInverse * inputPose;
		inputPose.translation() /= m_translationScale;
		result->poses().set("inputPose", inputPose);
	}

	Vector3d inputLinearVelocity;
	if (dataToFilter.vectors().get("inputLinearVelocity", &inputLinearVelocity))
	{
		inputLinearVelocity = m_transformInverse.linear() * inputLinearVelocity;
		inputLinearVelocity /= m_translationScale;
		result->vectors().set("linearVelocity", inputLinearVelocity);
	}

	// The force and torque must be similarly transformed into device space.  In order to reliably display the desired
	// forces as calculated by the simulation, the nominal forces and torques are not scaled by the translation scaling.
	// The benefit is that increasing the translation scaling does not result in larger penetrations for the
	// same force, but the downside is that as the translation scaling increases it becomes more likely that the haptic
	// feedback loop at a surface will become "active" (smaller motions penetrating another object are sufficient to
	// create forces ejecting the device's collision representation).  Therefore, a device that is having its
	// translation scaled may required a force scaling filter to reduce the forces.
	Vector3d force;
	if (dataToFilter.vectors().get("force", &force))
	{
		force = m_transformInverse.linear() * force;
		result->vectors().set("force", force);
	}

	Vector3d torque;
	if (dataToFilter.vectors().get("torque", &torque))
	{
		torque = m_transformInverse.linear() * torque;
		result->vectors().set("torque", torque);
	}

	// The Jacobians must be transformed into device space.  The Jacobians are scaled based on the translation scaling,
	// so that the forces displayed by the device will be correct for the scene-space motions, not for the device-space
	// motions.
	SurgSim::DataStructures::DataGroup::DynamicMatrixType springJacobian;
	if (dataToFilter.matrices().get("springJacobian", &springJacobian))
	{
		springJacobian.block<3,3>(0, 0).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(3, 0).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(0, 3).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(3, 3).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<6,3>(0, 0) *= m_translationScale;
		result->matrices().set("springJacobian", springJacobian);
	}

	SurgSim::DataStructures::DataGroup::DynamicMatrixType damperJacobian;
	if (dataToFilter.matrices().get("damperJacobian", &damperJacobian))
	{
		damperJacobian.block<3,3>(0, 0).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(3, 0).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(0, 3).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(3, 3).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<6,3>(0, 0) *= m_translationScale;
		result->matrices().set("damperJacobian", damperJacobian);
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
