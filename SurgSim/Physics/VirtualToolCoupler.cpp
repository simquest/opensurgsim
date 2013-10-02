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

#include <SurgSim/Physics/VirtualToolCoupler.h>

#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/Framework/Logger.h>
#include <SurgSim/Input/InputComponent.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Physics/RigidRepresentation.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Quaterniond;


namespace SurgSim{

namespace Physics{

Vector3d computeRotationVector(const RigidTransform3d& t1, const RigidTransform3d& t2)
{
	Quaterniond q1(t1.linear());
	Quaterniond q2(t2.linear());
	double angle;
	Vector3d axis;
	SurgSim::Math::computeAngleAndAxis((q1 * q2.inverse()).normalized(), &angle, &axis);
	return angle*axis;
}

VirtualToolCoupler::VirtualToolCoupler(const std::string& name, std::shared_ptr<SurgSim::Input::InputComponent> input,
						 std::shared_ptr<SurgSim::Physics::RigidRepresentation> rigid,
						 const std::string& poseName) :
	SurgSim::Framework::Behavior(name),
	m_input(input),
	m_rigid(rigid),
	m_poseName(poseName),
	m_linearStiffness(0.0),
	m_linearDamping(0.0),
	m_angularStiffness(0.0),
	m_angularDamping(0.0)
{
}

VirtualToolCoupler::~VirtualToolCoupler()
{
}

void VirtualToolCoupler::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_input->getData(&dataGroup);
	RigidTransform3d inputPose;
	dataGroup.poses().get(m_poseName, &inputPose);

	RigidTransform3d previousInputPose = m_previousState.getPose();
	Vector3d inputLinearVelocity = (inputPose.translation() - previousInputPose.translation()) / dt;
	Vector3d inputAngularVelocity = computeRotationVector(inputPose, previousInputPose) / dt;

	RigidRepresentationState objectState(m_rigid->getCurrentState());
	RigidTransform3d objectPose(objectState.getPose());
	Vector3d objectLinearVelocity(objectState.getLinearVelocity());
	Vector3d objectAnglularVelocity(objectState.getAngularVelocity());

	Vector3d force;
	force = m_linearStiffness * (inputPose.translation() - objectPose.translation());
	force += m_linearDamping * (inputLinearVelocity - objectLinearVelocity);

	Vector3d torque;
	torque = m_angularStiffness * computeRotationVector(inputPose, objectPose);
	torque += m_angularDamping * (inputAngularVelocity - objectAnglularVelocity);
	
	const Matrix33d identity3x3 = Matrix33d::Identity();
	m_rigid->addExternalForce(force, m_linearStiffness*identity3x3, m_linearDamping*identity3x3);
	m_rigid->addExternalTorque(torque, m_angularStiffness*identity3x3, m_angularDamping*identity3x3);

	m_previousState.setPose(inputPose);
	m_previousState.setLinearVelocity(inputLinearVelocity);
	m_previousState.setAngularVelocity(inputAngularVelocity);
}

bool VirtualToolCoupler::doInitialize()
{
	return true;
}

bool VirtualToolCoupler::doWakeUp()
{
	m_previousState = m_rigid->getCurrentState();
	return true;
}

int VirtualToolCoupler::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

void VirtualToolCoupler::setLinearStiffness(double linearStiffness)
{
	m_linearStiffness = linearStiffness;
}

void VirtualToolCoupler::setLinearDamping(double linearDamping)
{
	m_linearDamping = linearDamping;
}

void VirtualToolCoupler::setAngularStiffness(double angularStiffness)
{
	m_angularStiffness = angularStiffness;
}

void VirtualToolCoupler::setAngularDamping(double angularDamping)
{
	m_angularDamping = angularDamping;
}


}; /// Physics

}; /// SurgSim
