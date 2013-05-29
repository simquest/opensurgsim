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

#include <SurgSim/Physics/Actors/RigidActor.h>

#include <SurgSim/Math/Valid.h>
#include <SurgSim/Math/Quaternion.h>

namespace SurgSim{

namespace Physics{

RigidActor::RigidActor(const std::string& name)
	: RigidActorBase(name)
{
	// Initialize the number of degrees of freedom
	// 6 for a rigid body velocity-based (linear and angular velocities are the Dof)
	setNumDof(6);
}

RigidActor::~RigidActor()
{
}

void RigidActor::beforeUpdate(double dt)
{
}

void RigidActor::update(double dt)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	// For convenience
	RigidActorParameters& p = m_currentParameters;
	Vector3d         G = m_currentState.getPose().translation();
	Vector3d        dG = m_currentState.getLinearVelocity();
	Matrix33d        R = m_currentState.getPose().rotation();
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

	// Integrate the velocities to get the rigid actor pose
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

	// If something went wrong, we deactivate the actor
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

void RigidActor::afterUpdate(double dt)
{
}

void RigidActor::computeComplianceMatrix(double dt)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	m_C.setZero();

	// Compliance matrix for interactions:
	// C = ( Mlinear^-1       0     )
	//     (   0        Mangular^-1 )
	// with Mlinear^-1  = Id33.(1/m)/(1/dt + alphaLinear)]
	// with Mangular^-1 = I^-1      /(1/dt + alphaAngular)
	RigidActorParameters& p = m_currentParameters;
	double coefLin = (1.0/p.getMass()) / (1.0/dt + p.getLinearDamping());
	double coefAng =        1.0        / (1.0/dt + p.getAngularDamping());

	m_C(0,0) = m_C(1,1) = m_C(2,2) = coefLin;
	m_C.block<3,3>(3,3) = m_invGlobalInertia * coefAng;
}

void RigidActor::updateGlobalInertiaMatrices(const RigidActorState& state)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	const SurgSim::Math::Matrix33d& R = state.getPose().rotation();
	m_globalInertia =  R * m_currentParameters.getLocalInertia() * R.transpose();
	m_invGlobalInertia = m_globalInertia.inverse();
}

}; /// Physics

}; /// SurgSim
