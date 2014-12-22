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

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/LogMacros.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Math::makeSkewSymmetricMatrix;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector6d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix66d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Framework::checkAndConvert;

namespace SurgSim
{

namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::VirtualToolCoupler, VirtualToolCoupler);

VirtualToolCoupler::VirtualToolCoupler(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_poseName(SurgSim::DataStructures::Names::POSE),
	m_linearStiffness(std::numeric_limits<double>::quiet_NaN()),
	m_linearDamping(std::numeric_limits<double>::quiet_NaN()),
	m_angularStiffness(std::numeric_limits<double>::quiet_NaN()),
	m_angularDamping(std::numeric_limits<double>::quiet_NaN()),
	m_localAttachmentPoint(Vector3d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
									std::numeric_limits<double>::quiet_NaN())),
	m_calculateInertialTorques(false),
	m_hadCollisions(20),
	m_previouslyMovedRigidToInput(false),
	m_poseIndex(-1),
	m_linearVelocityIndex(-1),
	m_angularVelocityIndex(-1),
	m_forceIndex(-1),
	m_torqueIndex(-1),
	m_inputLinearVelocityIndex(-1),
	m_inputAngularVelocityIndex(-1),
	m_inputPoseIndex(-1),
	m_springJacobianIndex(-1),
	m_damperJacobianIndex(-1)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, SurgSim::DataStructures::OptionalValue<double>,
									  LinearStiffness, getOptionalLinearStiffness, setOptionalLinearStiffness);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, SurgSim::DataStructures::OptionalValue<double>,
									  LinearDamping, getOptionalLinearDamping, setOptionalLinearDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, SurgSim::DataStructures::OptionalValue<double>,
									  AngularStiffness, getOptionalAngularStiffness, setOptionalAngularStiffness);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, SurgSim::DataStructures::OptionalValue<double>,
									  AngularDamping, getOptionalAngularDamping, setOptionalAngularDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler,
									  SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>, AttachmentPoint,
									  getOptionalAttachmentPoint, setOptionalAttachmentPoint);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, bool, CalculateInertialTorques,
									  getCalculateInertialTorques, setCalculateInertialTorques);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, std::shared_ptr<SurgSim::Framework::Component>,
									  Input, getInput, setInput);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, std::shared_ptr<SurgSim::Framework::Component>,
									  Output, getOutput, setOutput);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VirtualToolCoupler, std::shared_ptr<SurgSim::Framework::Component>,
									  Representation, getRepresentation, setRepresentation);
}

VirtualToolCoupler::~VirtualToolCoupler()
{
}

const std::shared_ptr<SurgSim::Framework::Component> VirtualToolCoupler::getInput()
{
	return m_input;
}

void VirtualToolCoupler::setInput(const std::shared_ptr<SurgSim::Framework::Component> input)
{
	m_input = checkAndConvert<SurgSim::Input::InputComponent>(input, "SurgSim::Input::InputComponent");
}

const std::shared_ptr<SurgSim::Framework::Component> VirtualToolCoupler::getOutput()
{
	return m_output;
}

void VirtualToolCoupler::setOutput(const std::shared_ptr<SurgSim::Framework::Component> output)
{
	m_output = checkAndConvert<SurgSim::Input::OutputComponent>(output, "SurgSim::Input::OutputComponent");
}

const std::shared_ptr<SurgSim::Framework::Component> VirtualToolCoupler::getRepresentation()
{
	return m_rigid;
}

void VirtualToolCoupler::setRepresentation(const std::shared_ptr<SurgSim::Framework::Component> rigid)
{
	m_rigid = std::dynamic_pointer_cast<SurgSim::Physics::RigidRepresentation>(rigid);
}

const std::string& VirtualToolCoupler::getPoseName()
{
	return m_poseName;
}

void VirtualToolCoupler::setPoseName(const std::string& poseName)
{
	m_poseName = poseName;
}

void VirtualToolCoupler::update(double dt)
{
	SurgSim::DataStructures::DataGroup inputData;
	m_input->getData(&inputData);
	inputData.poses().cacheIndex(m_poseName, &m_poseIndex);
	inputData.vectors().cacheIndex(SurgSim::DataStructures::Names::LINEAR_VELOCITY, &m_linearVelocityIndex);
	inputData.vectors().cacheIndex(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, &m_angularVelocityIndex);

	RigidTransform3d inputPose;
	if (inputData.poses().get(m_poseIndex, &inputPose))
	{
		Vector3d inputLinearVelocity(Vector3d::Zero());
		inputData.vectors().get(m_linearVelocityIndex, &inputLinearVelocity);
		Vector3d inputAngularVelocity(Vector3d::Zero());
		inputData.vectors().get(m_angularVelocityIndex, &inputAngularVelocity);
		RigidTransform3d inputAlignment = m_input->getLocalPose();
		inputPose = inputAlignment * inputPose;
		inputLinearVelocity = inputAlignment.linear() * inputLinearVelocity;
		inputAngularVelocity = inputAlignment.rotation() * inputAngularVelocity;

		const RigidRepresentationState& objectState(m_rigid->getCurrentState());
		const RigidTransform3d objectPose(objectState.getPose());
		Vector3d rotationVector;
		SurgSim::Math::computeRotationVector(inputPose, objectPose, &rotationVector);
		Vector3d force;
		Vector3d torque;
		Matrix66d generalizedStiffness;
		Matrix66d generalizedDamping;

		bool moveRigidToInput = false;
		if (m_collision != nullptr)
		{
			m_hadCollisions.push_back(0 < m_collision->getCollisions().unsafeGet().size());
			moveRigidToInput = std::none_of(m_hadCollisions.begin(), m_hadCollisions.end(),
				[](bool collision){return collision;});
		}
		if (moveRigidToInput)
		{
			m_rigid->setLinearVelocity((inputPose.translation() -
				inputPose.linear() * m_localAttachmentPoint - objectPose.translation()) / dt);
			m_rigid->setAngularVelocity(rotationVector / dt);
			force = Vector3d::Zero();
			torque = Vector3d::Zero();
			m_previouslyMovedRigidToInput = true;
		}
		else
		{
			if (m_previouslyMovedRigidToInput)
			{
				m_rigid->setLinearVelocity(Vector3d::Zero());
				m_rigid->setAngularVelocity(Vector3d::Zero());
				m_previouslyMovedRigidToInput = false;
			}
			const Vector3d objectPosition = objectPose * m_rigid->getMassCenter();
			const Vector3d attachmentPoint = objectPose * m_localAttachmentPoint;
			Vector3d leverArm = attachmentPoint - objectPosition;
			Vector3d attachmentPointVelocity = objectState.getLinearVelocity();
			attachmentPointVelocity += objectState.getAngularVelocity().cross(leverArm);

			force = m_linearStiffness * (inputPose.translation() - attachmentPoint);
			force += m_linearDamping * (inputLinearVelocity - attachmentPointVelocity);

			torque = m_angularStiffness * rotationVector;
			torque += m_angularDamping * (inputAngularVelocity - objectState.getAngularVelocity());

			Vector6d generalizedForce;
			generalizedForce << force, torque;

			const Matrix33d identity3x3 = Matrix33d::Identity();
			const Matrix33d zero3x3 = Matrix33d::Zero();
			generalizedStiffness << m_linearStiffness * identity3x3, zero3x3,
				zero3x3                        , m_angularStiffness * identity3x3;
			generalizedDamping << m_linearDamping * identity3x3, zero3x3,
				zero3x3                      , m_angularDamping * identity3x3;

			if (m_calculateInertialTorques)
			{
				SurgSim::DataStructures::Location location;
				location.rigidLocalPosition.setValue(m_localAttachmentPoint);
				m_rigid->addExternalGeneralizedForce(location, generalizedForce, generalizedStiffness,
					generalizedDamping);
			}
			else
			{
				m_rigid->addExternalGeneralizedForce(generalizedForce, generalizedStiffness, generalizedDamping);
			}
		}

		if (m_output != nullptr)
		{
			RigidTransform3d outputAlignment = m_output->getLocalPose().inverse();
			Matrix33d outputAlignmentUnScaled = outputAlignment.rotation();

			m_outputData.poses().set(m_inputPoseIndex, outputAlignment * inputPose);
			m_outputData.vectors().set(m_inputLinearVelocityIndex, outputAlignment.linear() * inputLinearVelocity);
			m_outputData.vectors().set(m_inputAngularVelocityIndex, outputAlignmentUnScaled * inputAngularVelocity);

			if (moveRigidToInput)
			{
				m_outputData.vectors().reset(m_forceIndex);
				m_outputData.vectors().reset(m_torqueIndex);
				m_outputData.matrices().reset(m_springJacobianIndex);
				m_outputData.matrices().reset(m_damperJacobianIndex);
			}
			else
			{
				m_outputData.vectors().set(m_forceIndex, outputAlignmentUnScaled * (-force));
				m_outputData.vectors().set(m_torqueIndex, outputAlignmentUnScaled * (-torque));
				m_outputData.matrices().set(m_springJacobianIndex, -generalizedStiffness);
				m_outputData.matrices().set(m_damperJacobianIndex, -generalizedDamping);
			}

			m_output->setData(m_outputData);
		}
	}
}

bool VirtualToolCoupler::doInitialize()
{
	m_outputData = buildOutputData();

	m_forceIndex = m_outputData.vectors().getIndex(SurgSim::DataStructures::Names::FORCE);
	m_torqueIndex = m_outputData.vectors().getIndex(SurgSim::DataStructures::Names::TORQUE);
	m_inputLinearVelocityIndex = m_outputData.vectors().getIndex(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY);
	m_inputAngularVelocityIndex =
		m_outputData.vectors().getIndex(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY);
	m_inputPoseIndex = m_outputData.poses().getIndex(SurgSim::DataStructures::Names::INPUT_POSE);
	m_springJacobianIndex = m_outputData.matrices().getIndex(SurgSim::DataStructures::Names::SPRING_JACOBIAN);
	m_damperJacobianIndex = m_outputData.matrices().getIndex(SurgSim::DataStructures::Names::DAMPER_JACOBIAN);

	return true;
}

SurgSim::DataStructures::DataGroup VirtualToolCoupler::buildOutputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);
	builder.addVector(SurgSim::DataStructures::Names::TORQUE);
	builder.addMatrix(SurgSim::DataStructures::Names::SPRING_JACOBIAN);
	builder.addPose(SurgSim::DataStructures::Names::INPUT_POSE);
	builder.addMatrix(SurgSim::DataStructures::Names::DAMPER_JACOBIAN);
	builder.addVector(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY);
	builder.addVector(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY);
	return builder.createData();
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

	if (m_optionalAttachmentPoint.hasValue())
	{
		m_localAttachmentPoint = m_optionalAttachmentPoint.getValue();
	}
	else
	{
		m_localAttachmentPoint = m_rigid->getMassCenter();
	}

	// Provide sensible defaults based on the rigid representation.
	// If one or both of the stiffness and damping are not provided, they are
	// calculated to provide a critically damped system (dampingRatio-1.0).
	// For a mass-spring system, the damping ratio is defined as:
	//
	//     dampingRatio = (damping) / (2 * sqrt(mass * stiffness))
	//
	double dampingRatio = 1.0;
	double mass = m_rigid->getMass();
	if (!m_optionalLinearDamping.hasValue())
	{
		if (m_optionalLinearStiffness.hasValue())
		{
			m_linearStiffness = m_optionalLinearStiffness.getValue();
		}
		else
		{
			m_linearStiffness = mass * 100.0;
		}
		m_linearDamping = 2.0 * dampingRatio * sqrt(mass * m_linearStiffness);
	}
	else
	{
		m_linearDamping = m_optionalLinearDamping.getValue();
		if (m_optionalLinearStiffness.hasValue())
		{
			m_linearStiffness = m_optionalLinearStiffness.getValue();
		}
		else
		{
			m_linearStiffness = pow(m_linearDamping / dampingRatio, 2) / (4.0 * mass);
		}
	}

	Matrix33d leverArmMatrix = makeSkewSymmetricMatrix((m_localAttachmentPoint - m_rigid->getMassCenter()).eval());
	const Matrix33d& inertia = m_rigid->getLocalInertia() + mass * leverArmMatrix * leverArmMatrix;
	double maxInertia = inertia.eigenvalues().real().maxCoeff();
	if (!m_optionalAngularDamping.hasValue())
	{
		if (m_optionalAngularStiffness.hasValue())
		{
			m_angularStiffness = m_optionalAngularStiffness.getValue();
		}
		else
		{
			m_angularStiffness = maxInertia * 1000.0;
		}
		m_angularDamping = 2.0 * dampingRatio * sqrt(maxInertia * m_angularStiffness);
	}
	else
	{
		m_angularDamping = m_optionalAngularDamping.getValue();
		if (m_optionalAngularStiffness.hasValue())
		{
			m_angularStiffness = m_optionalAngularStiffness.getValue();
		}
		else
		{
			m_angularStiffness = pow(m_angularDamping / dampingRatio, 2) / (4.0 * maxInertia);
		}
	}


	return true;
}

int VirtualToolCoupler::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

void VirtualToolCoupler::overrideLinearStiffness(double linearStiffness)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot override vtc parameter after it has initialized";

	m_optionalLinearStiffness.setValue(linearStiffness);
	m_linearStiffness = linearStiffness;
}

double VirtualToolCoupler::getLinearStiffness()
{
	SURGSIM_ASSERT(isAwake() || m_optionalLinearStiffness.hasValue())
			<< "Vtc parameter has not been initialized";

	return m_linearStiffness;
}

void VirtualToolCoupler::overrideLinearDamping(double linearDamping)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot override vtc parameter after it has initialized";

	m_optionalLinearDamping.setValue(linearDamping);
	m_linearDamping = linearDamping;
}

double VirtualToolCoupler::getLinearDamping()
{
	SURGSIM_ASSERT(isAwake() || m_optionalLinearDamping.hasValue())
			<< "Vtc parameter has not been initialized";

	return m_linearDamping;
}

void VirtualToolCoupler::overrideAngularStiffness(double angularStiffness)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot override vtc parameter after it has initialized";

	m_optionalAngularStiffness.setValue(angularStiffness);
	m_angularStiffness = angularStiffness;
}

double VirtualToolCoupler::getAngularStiffness()
{
	SURGSIM_ASSERT(isAwake() || m_optionalAngularStiffness.hasValue())
			<< "Vtc parameter has not been initialized";

	return m_angularStiffness;
}

void VirtualToolCoupler::overrideAngularDamping(double angularDamping)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot override vtc parameter after it has initialized";

	m_optionalAngularDamping.setValue(angularDamping);
	m_angularDamping = angularDamping;
}

double VirtualToolCoupler::getAngularDamping()
{
	SURGSIM_ASSERT(isAwake() || m_optionalAngularDamping.hasValue())
			<< "Vtc parameter has not been initialized";

	return m_angularDamping;
}

void VirtualToolCoupler::overrideAttachmentPoint(const SurgSim::Math::Vector3d& attachment)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot override vtc parameter after it has initialized";

	m_optionalAttachmentPoint.setValue(attachment);
	m_localAttachmentPoint = attachment;
}

const SurgSim::Math::Vector3d& VirtualToolCoupler::getAttachmentPoint()
{
	SURGSIM_ASSERT(isAwake() || m_optionalAttachmentPoint.hasValue())
			<< "Vtc parameter has not been initialized";
	return m_localAttachmentPoint;
}

void VirtualToolCoupler::setOptionalLinearStiffness(
	const SurgSim::DataStructures::OptionalValue<double>& linearStiffness)
{
	m_optionalLinearStiffness = linearStiffness;
}

const SurgSim::DataStructures::OptionalValue<double>& VirtualToolCoupler::getOptionalLinearStiffness() const
{
	return m_optionalLinearStiffness;
}

void VirtualToolCoupler::setOptionalLinearDamping(const SurgSim::DataStructures::OptionalValue<double>& linearDamping)
{
	m_optionalLinearDamping = linearDamping;
}

const SurgSim::DataStructures::OptionalValue<double>& VirtualToolCoupler::getOptionalLinearDamping() const
{
	return m_optionalLinearDamping;
}

void VirtualToolCoupler::setOptionalAngularStiffness(
	const SurgSim::DataStructures::OptionalValue<double>& angularStiffness)
{
	m_optionalAngularStiffness = angularStiffness;
}

const SurgSim::DataStructures::OptionalValue<double>& VirtualToolCoupler::getOptionalAngularStiffness() const
{
	return m_optionalAngularStiffness;
}

void VirtualToolCoupler::setOptionalAngularDamping(
	const SurgSim::DataStructures::OptionalValue<double>& angularDamping)
{
	m_optionalAngularDamping = angularDamping;
}

const SurgSim::DataStructures::OptionalValue<double>& VirtualToolCoupler::getOptionalAngularDamping() const
{
	return m_optionalAngularDamping;
}

void VirtualToolCoupler::setOptionalAttachmentPoint(
	const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>& attachmentPoint)
{
	m_optionalAttachmentPoint = attachmentPoint;
}

const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>&
VirtualToolCoupler::getOptionalAttachmentPoint() const
{
	return m_optionalAttachmentPoint;
}

void VirtualToolCoupler::setCalculateInertialTorques(bool calculateInertialTorques)
{
	m_calculateInertialTorques = calculateInertialTorques;
}

bool VirtualToolCoupler::getCalculateInertialTorques() const
{
	return m_calculateInertialTorques;
}

void VirtualToolCoupler::setCollisionRepresentation(const std::shared_ptr<SurgSim::Framework::Component> collision)
{
	m_collision = std::dynamic_pointer_cast<SurgSim::Collision::Representation>(collision);
}

}; /// Physics
}; /// SurgSim
