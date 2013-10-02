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

#include <SurgSim/Framework/Logger.h>

#include <SurgSim/Physics/RigidRepresentation.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Valid.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Physics/Localization.h>
#include <SurgSim/Collision/Location.h>

using SurgSim::Collision::Location;

namespace SurgSim
{
namespace Physics
{

RigidRepresentation::RigidRepresentation(const std::string& name) :
	RigidRepresentationBase(name),
	m_externalForce(SurgSim::Math::Vector3d::Zero()),
	m_externalTorque(SurgSim::Math::Vector3d::Zero()),
	m_externalStiffnessMatrix(ComplianceMatrixType::Zero()),
	m_externalDampingMatrix(ComplianceMatrixType::Zero())
{
	// Initialize the number of degrees of freedom
	// 6 for a rigid body velocity-based (linear and angular velocities are the Dof)
	setNumDof(6);
}

RigidRepresentation::~RigidRepresentation()
{
}

SurgSim::Physics::RepresentationType RigidRepresentation::getType() const
{
	return REPRESENTATION_TYPE_RIGID;
}

void RigidRepresentation::setInitialParameters(const RigidRepresentationParameters& parameters)
{
	m_initialParameters = parameters;
	m_currentParameters = parameters;

	updateGlobalInertiaMatrices(m_currentState);
}


void RigidRepresentation::setCurrentParameters(const RigidRepresentationParameters& parameters)
{
	m_currentParameters = parameters;
	updateGlobalInertiaMatrices(m_currentState);
}

void RigidRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
}

void RigidRepresentation::addExternalForce(const SurgSim::Math::Vector3d& force, const SurgSim::Math::Matrix33d& K,
										   const SurgSim::Math::Matrix33d& D)
{
	m_externalForce = force;
	m_externalStiffnessMatrix.block<3,3>(0,0) = K;
	m_externalDampingMatrix.block<3,3>(0,0) = D;
}

void RigidRepresentation::addExternalTorque(const SurgSim::Math::Vector3d& torque, const SurgSim::Math::Matrix33d& K,
											const SurgSim::Math::Matrix33d& D)
{
	m_externalTorque = torque;
	m_externalStiffnessMatrix.block<3,3>(3,3) = K;
	m_externalDampingMatrix.block<3,3>(3,3) = D;
}

void RigidRepresentation::beforeUpdate(double dt)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	m_previousState = m_currentState;
}

void RigidRepresentation::update(double dt)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	// For convenience
	RigidRepresentationParameters& p = m_currentParameters;
	Vector3d         G = m_currentState.getPose().translation();
	Vector3d        dG = m_currentState.getLinearVelocity();
	Matrix33d        R = m_currentState.getPose().linear();
	Quaterniond      q = Quaterniond(R);
	Vector3d         w = m_currentState.getAngularVelocity();
	Quaterniond     dq;
	double       qNorm = q.norm(); // Norm of q before normalization

	// Backup the state
	m_previousState = m_currentState;

	// Rigid body dynamics (using backward euler numerical integration scheme):
	// { Id33.m.(v(t+dt) - v(t))/dt = f
	// { I     .(w(t+dt) - w(t))/dt = t - w(t)^(I.w(t))
	//
	// Integrating the Rayleigh damping on the velocity level:
	// { Id33.m.(1/dt + alphaLinear ).v(t+dt) = Id33.m.v(t)/dt + f
	// { I     .(1/dt + alphaAngular).w(t+dt) = I.w(t)/dt + t - w(t)^(I.w(t))

	// Compute external forces/torques
	m_force.setZero();
	m_torque.setZero();
	m_force += m_externalForce;
	m_torque += m_externalTorque;
	if (isGravityEnabled())
	{
		m_force += getGravity() * p.getMass();
	}
	m_torque -= w.cross(m_globalInertia * w);

	// Add Backward Euler RHS terms
	m_force  += p.getMass()     * dG / dt;
	m_torque += m_globalInertia *  w / dt;

	// Solve the 6D system on the velocity level
	{
		// { Id33.m.(1/dt + alphaLinear ).v(t+dt) = Id33.m.v(t)/dt + f
		// { I     .(1/dt + alphaAngular).w(t+dt) = I.w(t)/dt + t - w(t)^(I.w(t))
		dG = (1.0 / p.getMass())* m_force  / (1.0/dt + p.getLinearDamping());
		w  = m_invGlobalInertia * m_torque / (1.0/dt + p.getAngularDamping());
		// Compute the quaternion velocity as well: dq = 1/2.(0 w).q
		dq = Quaterniond(0.0, w[0], w[1], w[2]) * q;
		dq.coeffs() *= 0.5;
	}
	m_currentState.setLinearVelocity(dG);
	m_currentState.setAngularVelocity(w);

	// Integrate the velocities to get the rigid representation pose
	{
		// G(t+dt) = G(t) + dt. dG(t+dt)
		G += dG * dt;
		// q(t+dt) = q(t) + dt. dq(t+dt)
		Quaterniond dq_dt = dq;
		dq_dt.coeffs() *= dt;
		q.coeffs() += dq_dt.coeffs();
		// Normalize the quaternion to make sure we do have a rotation
		q.normalize();
	}
	m_currentState.setPose(SurgSim::Math::makeRigidTransform(q, G));

	// Compute the global inertia matrix with the current state
	updateGlobalInertiaMatrices(m_currentState);

	// If something went wrong, we deactivate the representation
	bool condition = SurgSim::Math::isValid(G);
	condition &= qNorm != 0.0;
	condition &= SurgSim::Math::isValid(q);
	condition &= fabs(1.0 - q.norm()) < 1e-3;
	SURGSIM_LOG_IF(! condition, SurgSim::Framework::Logger::getDefaultLogger(), DEBUG) << getName() <<
		" deactivated and reset because:" << std::endl << "m_G=(" <<
		G[0] << "," << G[1] << "," << G[2] << ") " <<
		"m_q=(" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ") " <<
		"|q| before normalization=" << qNorm << " " <<
		"|q| after normalization="<< q.norm() << std::endl;
	if (! condition)
	{
		resetState();
		setIsActive(false);
	}

	// Prepare the compliance matrix
	computeComplianceMatrix(dt);
}

void RigidRepresentation::afterUpdate(double dt)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	m_finalState = m_currentState;
	m_externalForce = SurgSim::Math::Vector3d::Zero();
	m_externalTorque = SurgSim::Math::Vector3d::Zero();
	m_externalStiffnessMatrix = ComplianceMatrixType::Zero();
	m_externalDampingMatrix = ComplianceMatrixType::Zero();
}

void RigidRepresentation::applyDofCorrection(
	double dt,
	const Eigen::VectorBlock<SurgSim::Math::MlcpSolution::Vector>& dofCorrection)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	Vector3d          G = m_currentState.getPose().translation();
	Vector3d         dG = m_currentState.getLinearVelocity();
	Matrix33d         R = m_currentState.getPose().linear();
	Quaterniond       q = Quaterniond(R);
	Vector3d          w = m_currentState.getAngularVelocity();

	const SurgSim::Math::Vector3d& delta_dG = dofCorrection.segment(0,3);
	const SurgSim::Math::Vector3d& delta_w  = dofCorrection.segment(3,3);
	Quaterniond delta_dq = Quaterniond(delta_w[0],delta_w[1],delta_w[2],0.0) * q;
	delta_dq.coeffs() *= 0.5;

	dG += delta_dG;
	w  += delta_w;
	m_currentState.setLinearVelocity(dG);
	m_currentState.setAngularVelocity(w);

	// Integrate the velocities to get the rigid representation pose
	{
		// G(t+dt) = G(t) + dt. delta_dG(t+dt)
		G += delta_dG * dt;
		// q(t+dt) = q(t) + dt. delta_dq(t+dt)
		q.coeffs() += delta_dq.coeffs() * dt;
		// Normalize the quaternion to make sure we do have a rotation
		q.normalize();
	}
	m_currentState.setPose(SurgSim::Math::makeRigidTransform(q, G));

	// Compute the global inertia matrix with the current state
	updateGlobalInertiaMatrices(m_currentState);

	// If something went wrong, we deactivate the representation
	bool condition = SurgSim::Math::isValid(G);
	condition &= SurgSim::Math::isValid(q);
	condition &= fabs(1.0 - q.norm()) < 1e-3;
	SURGSIM_LOG_IF(! condition, SurgSim::Framework::Logger::getDefaultLogger(), DEBUG) << getName() <<
		" deactivated and reset because:" << std::endl << "m_G=(" <<
		G[0] << "," << G[1] << "," << G[2] << ") " <<
		"m_q=(" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ") " <<
		"|q| after normalization="<< q.norm() << std::endl;
	if (! condition)
	{
		resetState();
		setIsActive(false);
	}

	// Prepare the compliance matrix
	computeComplianceMatrix(dt);
}

void SurgSim::Physics::RigidRepresentation::resetParameters()
{
	Representation::resetParameters();
	m_currentParameters = m_initialParameters;

	updateGlobalInertiaMatrices(m_currentState);
}

const Eigen::Matrix<double, 6,6, Eigen::DontAlign | Eigen::RowMajor>&
	SurgSim::Physics::RigidRepresentation::getComplianceMatrix() const
{
	return m_C;
}

void RigidRepresentation::computeComplianceMatrix(double dt)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	ComplianceMatrixType systemMatrix;
	RigidRepresentationParameters& parameters = m_currentParameters;
	const SurgSim::Math::Matrix33d identity3x3 = SurgSim::Math::Matrix33d::Identity();
	systemMatrix.block<3,3>(0,0) = identity3x3 * (parameters.getMass() / dt + parameters.getLinearDamping());
	systemMatrix.block<3,3>(3,3) = m_globalInertia / dt + parameters.getAngularDamping() * identity3x3;
	systemMatrix += m_externalDampingMatrix + m_externalStiffnessMatrix * dt;

	m_C.setZero();

	//Invert systemMatrix
	//We can use this shortcut because we know the linear and angular terms are independant
	m_C.block<3,3>(0,0) = systemMatrix.block<3,3>(0,0).inverse();
	m_C.block<3,3>(3,3) = systemMatrix.block<3,3>(3,3).inverse();
}

void RigidRepresentation::updateGlobalInertiaMatrices(const RigidRepresentationState& state)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	const SurgSim::Math::Matrix33d& R = state.getPose().linear();
	m_globalInertia =  R * m_currentParameters.getLocalInertia() * R.transpose();
	m_invGlobalInertia = m_globalInertia.inverse();
}

}; /// Physics

}; /// SurgSim
