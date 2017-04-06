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

#include <boost/thread/locks.hpp>

#include "SurgSim/Devices/DeviceFilters/PoseTransform.h"

#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::UnalignedRigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::PoseTransform, PoseTransform);

PoseTransform::PoseTransform(const std::string& name) :
	DeviceFilter(name),
	m_transform(RigidTransform3d::Identity()),
	m_transformInverse(RigidTransform3d::Identity()),
	m_translationScale(1.0)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PoseTransform, double, TranslationScale,
		getTranslationScale, setTranslationScale);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PoseTransform, UnalignedRigidTransform3d, Transform, getTransform, setTransform);
}

void PoseTransform::filterInput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result)
{
	boost::lock_guard<boost::mutex> lock(m_mutex); // Prevent the transform or scaling from being set simultaneously.

	*result = dataToFilter;  // Pass on all the data entries.

	RigidTransform3d pose; // If there is a pose, scale the translation, then transform the result.
	if (dataToFilter.poses().get(DataStructures::Names::POSE, &pose))
	{
		pose.translation() *= m_translationScale;
		pose = m_transform * pose;
		result->poses().set(DataStructures::Names::POSE, pose);
	}

	// If there is a linear velocity, scale then rotate it.  The linear velocity is scaled because it is the change
	// in translation over time, and the translation is being scaled.
	Vector3d linearVelocity;
	if (dataToFilter.vectors().get(DataStructures::Names::LINEAR_VELOCITY, &linearVelocity))
	{
		linearVelocity *= m_translationScale;
		linearVelocity = m_transform.linear() * linearVelocity;
		result->vectors().set(DataStructures::Names::LINEAR_VELOCITY, linearVelocity);
	}

	Vector3d angularVelocity; // If there is an angular velocity, rotate it.
	if (dataToFilter.vectors().get(DataStructures::Names::ANGULAR_VELOCITY, &angularVelocity))
	{
		angularVelocity = m_transform.linear() * angularVelocity;
		result->vectors().set(DataStructures::Names::ANGULAR_VELOCITY, angularVelocity);
	}
}

void PoseTransform::filterOutput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result)
{
	boost::lock_guard<boost::mutex> lock(m_mutex); // Prevent the transform or scaling from being set simultaneously.

	*result = dataToFilter;  // Pass on all the data entries.

	// Since the haptic devices will compare the data in the output DataGroup to a raw input pose, the filter must
	// perform the reverse transform and scaling to data used by the haptic devices.

	// The force and torque must be transformed into device space.  In order to reliably display the desired
	// forces as calculated by the simulation, the nominal forces and torques are not scaled by the translation scaling.
	// The benefit is that increasing the translation scaling does not result in larger penetrations for the
	// same force, but the downside is that as the translation scaling increases it becomes more likely that the haptic
	// feedback loop at a surface will become "active" (smaller motions penetrating another object are sufficient to
	// create forces ejecting the device's collision representation).  Therefore, a device that is having its
	// translation scaled may required a force scaling filter to reduce the forces.
	Vector3d force;
	if (dataToFilter.vectors().get(DataStructures::Names::FORCE, &force))
	{
		force = m_transformInverse.linear() * force;
		result->vectors().set(DataStructures::Names::FORCE, force);
	}

	Vector3d torque;
	if (dataToFilter.vectors().get(DataStructures::Names::TORQUE, &torque))
	{
		torque = m_transformInverse.linear() * torque;
		result->vectors().set(DataStructures::Names::TORQUE, torque);
	}

	// The Jacobians must be transformed into device space.  The Jacobians are scaled based on the translation scaling,
	// so that the forces and torques displayed by the device will be correct for the scene-space motions, not for the
	// device-space motions.  Let R be the linear rotation portion of our transform, s be the translation scaling
	// factor, and J be the 3x3 upper-left corner of the spring Jacobian.  Then:
	// delta-translation-in-scene = s * R * delta-translation-in-device
	// delta-force-in-scene = J * delta-translation-in-scene
	// So to transform J into the device space, we left-multiply by R^-1, and right-multiply by R.
	// Then, because we want the forces to be scaled as in scene-space, we scale J by s.  The bottom-left 3x3 block
	// in springJacobian (or damperJacobian) also transforms from delta-translation and is treated the same, while the
	// two 3x3 blocks on the right half (of springJacobian or damperJacobian) will be multiplied by the delta-rotation
	// (not delta-translation) and so should not be scaled.
	DataGroup::DynamicMatrixType springJacobian;
	if (dataToFilter.matrices().get(DataStructures::Names::SPRING_JACOBIAN, &springJacobian))
	{
		springJacobian.block<3,3>(0, 0).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(0, 0).applyOnTheRight(m_transform.linear());
		springJacobian.block<3,3>(3, 0).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(3, 0).applyOnTheRight(m_transform.linear());
		springJacobian.block<3,3>(0, 3).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(0, 3).applyOnTheRight(m_transform.linear());
		springJacobian.block<3,3>(3, 3).applyOnTheLeft(m_transformInverse.linear());
		springJacobian.block<3,3>(3, 3).applyOnTheRight(m_transform.linear());
		springJacobian.block<6,3>(0, 0) *= m_translationScale;
		result->matrices().set(DataStructures::Names::SPRING_JACOBIAN, springJacobian);
	}

	RigidTransform3d inputPose;
	if (dataToFilter.poses().get(DataStructures::Names::INPUT_POSE, &inputPose))
	{
		inputPose = m_transformInverse * inputPose;
		inputPose.translation() /= m_translationScale;
		result->poses().set(DataStructures::Names::INPUT_POSE, inputPose);
	}

	DataGroup::DynamicMatrixType damperJacobian;
	if (dataToFilter.matrices().get(DataStructures::Names::DAMPER_JACOBIAN, &damperJacobian))
	{
		damperJacobian.block<3,3>(0, 0).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(0, 0).applyOnTheRight(m_transform.linear());
		damperJacobian.block<3,3>(3, 0).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(3, 0).applyOnTheRight(m_transform.linear());
		damperJacobian.block<3,3>(0, 3).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(0, 3).applyOnTheRight(m_transform.linear());
		damperJacobian.block<3,3>(3, 3).applyOnTheLeft(m_transformInverse.linear());
		damperJacobian.block<3,3>(3, 3).applyOnTheRight(m_transform.linear());
		damperJacobian.block<6,3>(0, 0) *= m_translationScale;
		result->matrices().set(DataStructures::Names::DAMPER_JACOBIAN, damperJacobian);
	}

	Vector3d inputLinearVelocity;
	if (dataToFilter.vectors().get(DataStructures::Names::INPUT_LINEAR_VELOCITY, &inputLinearVelocity))
	{
		inputLinearVelocity = m_transformInverse.linear() * inputLinearVelocity;
		inputLinearVelocity /= m_translationScale;
		result->vectors().set(DataStructures::Names::INPUT_LINEAR_VELOCITY, inputLinearVelocity);
	}

	Vector3d inputAngularVelocity;
	if (dataToFilter.vectors().get(DataStructures::Names::INPUT_ANGULAR_VELOCITY, &inputAngularVelocity))
	{
		inputAngularVelocity = m_transformInverse.linear() * inputAngularVelocity;
		result->vectors().set(DataStructures::Names::INPUT_ANGULAR_VELOCITY, inputAngularVelocity);
	}
}

double PoseTransform::getTranslationScale() const
{
	return m_translationScale;
}

void PoseTransform::setTranslationScale(double translationScale)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_translationScale = translationScale;
}

const SurgSim::Math::UnalignedRigidTransform3d& PoseTransform::getTransform() const
{
	return m_transform;
}

void PoseTransform::setTransform(const UnalignedRigidTransform3d& transform)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_transform = transform;
	m_transformInverse = m_transform.inverse();
}

};  // namespace Devices
};  // namespace SurgSim
