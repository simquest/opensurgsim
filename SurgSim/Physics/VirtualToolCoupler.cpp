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
#include "SurgSim/Framework/LogMacros.h"
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
using SurgSim::Math::Matrix66d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Quaterniond;


namespace SurgSim
{

namespace Physics
{

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
	builder.addVector("torque");
	builder.addMatrix("jacobianFromPosition");
	builder.addPose("inputPose");
	builder.addMatrix("jacobianFromVelocity");
	builder.addVector("inputLinearVelocity");
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

		RigidRepresentationState objectState(m_rigid->getCurrentState());
		RigidTransform3d objectPose(objectState.getPose());

		Vector3d force = m_linearStiffness * (inputPose.translation() - objectPose.translation());
		force += m_linearDamping * (inputLinearVelocity - objectState.getLinearVelocity());

		Vector3d rotationVector;
		SurgSim::Math::computeRotationVector(inputPose, objectPose, &rotationVector);
		Vector3d torque = m_angularStiffness * rotationVector;
		torque += m_angularDamping * (inputAngularVelocity - objectState.getAngularVelocity());

		const Matrix33d identity3x3 = Matrix33d::Identity();
		const Matrix33d linearStiffnessMatrix = m_linearStiffness * identity3x3;
		const Matrix33d linearDampingMatrix = m_linearDamping * identity3x3;
		const Matrix33d angularStiffnessMatrix = m_angularStiffness * identity3x3;
		const Matrix33d angularDampingMatrix = m_angularDamping * identity3x3;
		m_rigid->addExternalForce(force, linearStiffnessMatrix, linearDampingMatrix);
		m_rigid->addExternalTorque(torque, angularStiffnessMatrix, angularDampingMatrix);

		if (m_output != nullptr)
		{
			m_outputData.vectors().set("force", -force * m_outputForceScaling);
			m_outputData.vectors().set("torque", -torque * m_outputTorqueScaling);
			m_outputData.vectors().set("inputLinearVelocity", inputLinearVelocity);
			m_outputData.vectors().set("inputAngularVelocity", inputAngularVelocity);

			m_outputData.poses().set("inputPose", inputPose);

			Matrix66d jacobianFromPosition = Matrix66d::Zero();
			Matrix33d outputLinearStiffnessMatrix = -linearStiffnessMatrix * m_outputForceScaling;
			SurgSim::Math::setSubMatrix(outputLinearStiffnessMatrix, 0, 0, 3, 3, &jacobianFromPosition);
			Matrix33d outputAngularStiffnessMatrix = -angularStiffnessMatrix * m_outputTorqueScaling;
			SurgSim::Math::setSubMatrix(outputAngularStiffnessMatrix, 1, 1, 3, 3, &jacobianFromPosition);
			m_outputData.matrices().set("jacobianFromPosition", jacobianFromPosition);

			Matrix66d jacobianFromVelocity = Matrix66d::Zero();
			Matrix33d outputLinearDampingMatrix = -linearDampingMatrix * m_outputForceScaling;
			SurgSim::Math::setSubMatrix(outputLinearDampingMatrix, 0, 0, 3, 3, &jacobianFromVelocity);
			Matrix33d outputAngularDampingMatrix = -angularDampingMatrix * m_outputTorqueScaling;
			SurgSim::Math::setSubMatrix(outputAngularDampingMatrix, 1, 1, 3, 3, &jacobianFromVelocity);
			m_outputData.matrices().set("jacobianFromVelocity", jacobianFromVelocity);

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
	if (m_input == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << "VirtualToolCoupler named " <<
			getName() << " does not have an Input Component.";
		return false;
	}
	if (m_rigid == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << "VirtualToolCoupler named " <<
			getName() << " does not have a Representation.";
		return false;
	}
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
