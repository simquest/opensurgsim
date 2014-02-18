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

#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Quaterniond;


namespace SurgSim
{

namespace Physics
{

Vector3d computeRotationVector(const RigidTransform3d& t1, const RigidTransform3d& t2)
{
	Quaterniond q1(t1.linear());
	Quaterniond q2(t2.linear());
	double angle;
	Vector3d axis;
	SurgSim::Math::computeAngleAndAxis((q1 * q2.inverse()).normalized(), &angle, &axis);
	return angle * axis;
}

VirtualToolCoupler::VirtualToolCoupler(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_poseName("pose"),
	m_linearStiffness(0.0),
	m_linearDamping(0.0),
	m_angularStiffness(0.0),
	m_angularDamping(0.0),
	m_outputForceScaling(1.0),
	m_outputTorqueScaling(1.0)
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector("force");
	builder.addMatrix("forcePositionJacobian");
	builder.addVector("inputPosition");
	builder.addMatrix("forceLinearVelocityJacobian");
	builder.addVector("inputLinearVelocity");
	builder.addVector("torque");
	builder.addMatrix("torqueAngleJacobian");
	builder.addMatrix("inputOrientation");
	builder.addMatrix("torqueAngularVelocityJacobian");
	builder.addVector("inputAngularVelocity");
	m_outputData = builder.createData();
}

VirtualToolCoupler::~VirtualToolCoupler()
{
}

void VirtualToolCoupler::setInput(const std::shared_ptr<SurgSim::Input::InputComponent> input)
{
	m_input = input;
}

void VirtualToolCoupler::setOutput(const std::shared_ptr<SurgSim::Input::OutputComponent> output)
{
	m_output = output;
}

void VirtualToolCoupler::setRepresentation(const std::shared_ptr<SurgSim::Physics::RigidRepresentation> rigid)
{
	m_rigid = rigid;
}

void VirtualToolCoupler::setPoseName(const std::string& poseName)
{
	m_poseName = poseName;
}

void VirtualToolCoupler::update(double dt)
{
	SurgSim::DataStructures::DataGroup inputData;
	SURGSIM_ASSERT(m_input) << "VirtualToolCoupler named " << getName() << " does not have an Input Component.";
	m_input->getData(&inputData);
	RigidTransform3d inputPose;
	if (inputData.poses().get(m_poseName, &inputPose))
	{
		// TODO(ryanbeasley): If the RigidRepresentation is not colliding, we should turn off the VTC forces and set the
		// RigidRepresentation's state to the input state.
		Vector3d inputLinearVelocity, inputAngularVelocity;
		inputLinearVelocity.setZero();
		inputData.vectors().get("linearVelocity", &inputLinearVelocity);
		inputAngularVelocity.setZero();
		inputData.vectors().get("angularVelocity", &inputAngularVelocity);

		SURGSIM_ASSERT(m_rigid) << "VirtualToolCoupler named " << getName() << " does not have a Representation.";
		RigidRepresentationState objectState(m_rigid->getCurrentState());
		RigidTransform3d objectPose(objectState.getPose());

		Vector3d force = m_linearStiffness * (inputPose.translation() - objectPose.translation());
		force += m_linearDamping * (inputLinearVelocity - objectState.getLinearVelocity());

		Vector3d torque = m_angularStiffness * computeRotationVector(inputPose, objectPose);
		torque += m_angularDamping * (inputAngularVelocity - objectState.getAngularVelocity());

		const Matrix33d identity3x3 = Matrix33d::Identity();
		const Matrix33d linearStiffnessMatrix = m_linearStiffness * identity3x3;
		const Matrix33d linearDampingMatrix = m_linearDamping * identity3x3;
		const Matrix33d angularStiffnessMatrix = m_angularStiffness * identity3x3;
		const Matrix33d angularDampingMatrix = m_angularDamping * identity3x3;
		m_rigid->addExternalForce(force, linearStiffnessMatrix, linearDampingMatrix);
		m_rigid->addExternalTorque(torque, angularStiffnessMatrix, angularDampingMatrix);

		if (m_output)
		{
			m_outputData.vectors().set("force", -force * m_outputForceScaling);
			m_outputData.matrices().set("forcePositionJacobian", -linearStiffnessMatrix * m_outputForceScaling);
			m_outputData.vectors().set("inputPosition", inputPose.translation());
			m_outputData.matrices().set("forceVelocityJacobian", -linearDampingMatrix * m_outputForceScaling);
			m_outputData.vectors().set("inputLinearVelocity", inputLinearVelocity);
			m_outputData.vectors().set("torque", -force * m_outputForceScaling);
			m_outputData.matrices().set("torqueAngleJacobian", -angularStiffnessMatrix * m_outputForceScaling);
			m_outputData.matrices().set("inputOrientation", inputPose.linear());
			m_outputData.matrices().set("torqueAngularVelocityJacobian", -angularDampingMatrix * m_outputForceScaling);
			m_outputData.vectors().set("inputAngularVelocity", inputAngularVelocity);
			m_output->setData(m_outputData);
		}
	}
}

bool VirtualToolCoupler::doInitialize()
{
	return true;
}

bool VirtualToolCoupler::doWakeUp()
{
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

void VirtualToolCoupler::setOutputForceScaling(double forceScaling)
{
	m_outputForceScaling = forceScaling;
}

void VirtualToolCoupler::setOutputTorqueScaling(double torqueScaling)
{
	m_outputTorqueScaling = torqueScaling;
}

}; /// Physics
}; /// SurgSim
