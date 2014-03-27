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

#include <Eigen/Eigenvalues> 

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
	m_outputForceScaling(1.0),
	m_outputTorqueScaling(1.0)
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector("force");
	builder.addVector("torque");
	builder.addMatrix("springJacobian");
	builder.addPose("inputPose");
	builder.addMatrix("damperJacobian");
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

		Vector3d force = m_linearStiffness.getValue() * (inputPose.translation() - objectPose.translation());
		force += m_linearDamping.getValue() * (inputLinearVelocity - objectState.getLinearVelocity());

		Vector3d rotationVector;
		SurgSim::Math::computeRotationVector(inputPose, objectPose, &rotationVector);
		Vector3d torque = m_angularStiffness.getValue() * rotationVector;
		torque += m_angularDamping.getValue() * (inputAngularVelocity - objectState.getAngularVelocity());

		const Matrix33d identity3x3 = Matrix33d::Identity();
		const Matrix33d linearStiffnessMatrix = m_linearStiffness.getValue() * identity3x3;
		const Matrix33d linearDampingMatrix = m_linearDamping.getValue() * identity3x3;
		const Matrix33d angularStiffnessMatrix = m_angularStiffness.getValue() * identity3x3;
		const Matrix33d angularDampingMatrix = m_angularDamping.getValue() * identity3x3;
		m_rigid->addExternalForce(force, linearStiffnessMatrix, linearDampingMatrix);
		m_rigid->addExternalTorque(torque, angularStiffnessMatrix, angularDampingMatrix);

		if (m_output != nullptr)
		{
			m_outputData.vectors().set("force", -force * m_outputForceScaling);
			m_outputData.vectors().set("torque", -torque * m_outputTorqueScaling);
			m_outputData.vectors().set("inputLinearVelocity", inputLinearVelocity);
			m_outputData.vectors().set("inputAngularVelocity", inputAngularVelocity);

			m_outputData.poses().set("inputPose", inputPose);

			Matrix66d springJacobian = Matrix66d::Zero();
			Matrix33d outputLinearStiffnessMatrix = -linearStiffnessMatrix * m_outputForceScaling;
			SurgSim::Math::setSubMatrix(outputLinearStiffnessMatrix, 0, 0, 3, 3, &springJacobian);
			Matrix33d outputAngularStiffnessMatrix = -angularStiffnessMatrix * m_outputTorqueScaling;
			SurgSim::Math::setSubMatrix(outputAngularStiffnessMatrix, 1, 1, 3, 3, &springJacobian);
			m_outputData.matrices().set("springJacobian", springJacobian);

			Matrix66d damperJacobian = Matrix66d::Zero();
			Matrix33d outputLinearDampingMatrix = -linearDampingMatrix * m_outputForceScaling;
			SurgSim::Math::setSubMatrix(outputLinearDampingMatrix, 0, 0, 3, 3, &damperJacobian);
			Matrix33d outputAngularDampingMatrix = -angularDampingMatrix * m_outputTorqueScaling;
			SurgSim::Math::setSubMatrix(outputAngularDampingMatrix, 1, 1, 3, 3, &damperJacobian);
			m_outputData.matrices().set("damperJacobian", damperJacobian);

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

	//Provide sensible defaults based on the rigid representation
	double dampingRatio = 1.0;

	double mass = m_rigid->getInitialParameters().getMass();
	if (! m_linearDamping.hasValue())
	{
		if (! m_linearStiffness.hasValue())
		{
			m_linearStiffness.setValue(mass * 2000.0);
		}
		m_linearDamping.setValue(2.0 * dampingRatio * sqrt(mass * m_linearStiffness.getValue()));
	}
	else
	{
		if (! m_linearStiffness.hasValue())
		{
			m_linearStiffness.setValue(pow(m_linearDamping.getValue() / dampingRatio, 2) / (4.0 * mass));
		}
	}

	const Matrix33d& inertia = m_rigid->getInitialParameters().getLocalInertia();
	double maxInertia = inertia.eigenvalues().real().maxCoeff();
	if (! m_angularDamping.hasValue())
	{
		if (! m_angularStiffness.hasValue())
		{
			m_angularStiffness.setValue(maxInertia * 1000.0);
		}
		m_angularDamping.setValue(2.0 * dampingRatio * sqrt(maxInertia * m_angularStiffness.getValue()));
	}
	else
	{
		if (! m_angularStiffness.hasValue())
		{
			m_angularStiffness.setValue(pow(m_angularDamping.getValue() / dampingRatio, 2) / (4.0 * maxInertia));
		}
	}

	return true;
}

int VirtualToolCoupler::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

void VirtualToolCoupler::setLinearStiffness(double linearStiffness)
{
	m_linearStiffness.setValue(linearStiffness);
}

void VirtualToolCoupler::setLinearDamping(double linearDamping)
{
	m_linearDamping.setValue(linearDamping);
}

void VirtualToolCoupler::setAngularStiffness(double angularStiffness)
{
	m_angularStiffness.setValue(angularStiffness);
}

void VirtualToolCoupler::setAngularDamping(double angularDamping)
{
	m_angularDamping.setValue(angularDamping);
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
