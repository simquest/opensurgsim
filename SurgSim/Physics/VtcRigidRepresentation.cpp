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

#include <SurgSim/Physics/VtcRigidRepresentation.h>

#include <SurgSim/Math/Valid.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Quaternion.h>

namespace SurgSim{

namespace Physics{

VtcRigidRepresentation::VtcRigidRepresentation(const std::string& name)
	: RigidRepresentationBase(name)
{
	// Initialize the number of degrees of freedom
	// 6 for a rigid body velocity-based (linear and angular velocities are the Dof)
	setNumDof(6);
}

VtcRigidRepresentation::~VtcRigidRepresentation()
{
}

void VtcRigidRepresentation::beforeUpdate(double dt)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	// Vtc pose has been potentially set externally by calling setPose()
	// We need to update the vtc velocities at each time step
	{
		Vector3d p = m_currentVtcState.getPose().translation() - m_previousVtcState.getPose().translation();
		m_currentVtcState.setLinearVelocity(p / dt);

		const SurgSim::Math::Quaterniond q(m_currentVtcState.getPose().rotation());
		const SurgSim::Math::Quaterniond qprev(m_previousVtcState.getPose().rotation());
		SurgSim::Math::Quaterniond deltaq = q * qprev.inverse();
		double angle;
		Vector3d axis;
		SurgSim::Math::computeAngleAndAxis(deltaq, &angle, &axis);
		// NOTE: axis*angle = rotationVector
		m_currentVtcState.setAngularVelocity(axis * (angle / dt));
	}
}

void VtcRigidRepresentation::update(double dt)
{
	using namespace SurgSim::Framework;
	using SurgSim::Math::isValid;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	// For convenience
	RigidRepresentationParameters&  param = m_currentParameters;
	VtcRigidParameters& vtcParam = m_currentVtcParameters;
	Vector3d         G = m_currentState.getPose().translation();
	Vector3d        dG = m_currentState.getLinearVelocity();
	const Matrix33d& R = m_currentState.getPose().rotation();
	Quaterniond      q = Quaterniond(R);
	Vector3d         w = m_currentState.getAngularVelocity();
	Quaterniond     dq;
	double       qNorm; // Norm of q before normalization.

	// Backup current state and current vtc state
	m_previousState = m_currentState;
	m_previousVtcState = m_currentVtcState;

	// Developing the equations integrating the Rayleigh damping on the velocity level:
	// { Id33.m.(1/dt + alphaLinear ).v(t+dt) = m.v(t)/dt + f
	// { I     .(1/dt + alphaAngular).w(t+dt) = I.w(t)/dt + t - w(t)^(I.w(t))

	// Compute external forces/torques
	m_force.setZero();
	m_torque.setZero();
	if (isGravityEnabled())
	{
		m_force += getGravity() * param.getMass();
	}
	m_torque -= w.cross(m_globalInertia * w);

	// Vtc part
	// alphaLinearVtc  = Vtc linear  damping
	// alphaAngularVtc = Vtc angular damping
	// k_t = Vtc linear  stiffness
	// k_r = Vtc angular stiffness
	{
		//{ Id33.m.v(t+dt).[1/dt + alphaLinear  + alphaLinearVtc/m    + dt.k_t/m] =
		//                           = f                 + alphaLinearVtc.v(target)  + k_t.[x(target)-x(t)] + m.v(t)/dt
		//{ I     .w(t+dt).[1/dt + alphaAngular + I^-1.alphaAngularVtc          ] =
		//                           = t - w(t)^(I.w(t)) + alphaAngularVtc.w(target) + k_r.(alpha.u)        + I.w(t)/dt
		//
		// System matrix on the rotational DOF:
		//  Id33.m.(1/dt + alphaLinear  + alphaLinearVtc/m    + dt.k_t/m)
		// System matrix on the rotational DOF:
		//  I     .(1/dt + alphaAngular + I^-1.alphaAngularVtc)

		// Fpos = k_t.[x(target)-x(t)]
		Vector3d Fpos = (m_currentVtcState.getPose().translation() - G) * vtcParam.getVtcLinearStiffness();
		m_force  += Fpos;
		// Fvel = alphaLinearVtc.[v(t)]
		Vector3d Fvel = m_currentVtcState.getLinearVelocity() * vtcParam.getVtcLinearDamping();
		m_force  += Fvel;

		Quaterniond deltaq = Quaterniond(m_currentVtcState.getPose().rotation()) * q.inverse();
		deltaq.normalize();
		double angle;
		Vector3d axis;
		SurgSim::Math::computeAngleAndAxis(deltaq, &angle, &axis);
		// Tpos = k_r.[alpha.u] with (alpha,u) the axis and angle of qTarget.q^-1
		Vector3d Tpos = axis * (angle * vtcParam.getVtcAngularStiffness());
		m_torque += Tpos;
		// Tvel = alphaAngularVtc.[w(t)]
		Vector3d Tvel = m_currentVtcState.getAngularVelocity() * vtcParam.getVtcAngularDamping();
		m_torque += Tvel;
	}

	// Add Backward Euler RHS terms
	m_force  += param.getMass() * dG / dt;
	m_torque += m_globalInertia *  w / dt;

	// Solve the 6D system on the velocity level
	{
		//{ Id33.m.v(t+dt).[1/dt + alphaLinear  + alphaLinearVtc/m    + dt.k_t/m] =
		//                           = f                 + alphaLinearVtc.v(target)  + k_t.[x(target)-x(t)] + m.v(t)/dt
		//{ I     .w(t+dt).[1/dt + alphaAngular + I^-1.alphaAngularVtc          ] =
		//                           = t - w(t)^(I.w(t)) + alphaAngularVtc.w(target) + k_r.(alpha.u)        + I.w(t)/dt
		double invMass = 1.0 / param.getMass();
		dG = invMass * m_force  / (1.0 / dt + param.getLinearDamping() +
								   (vtcParam.getVtcLinearDamping() + dt * vtcParam.getVtcLinearStiffness()) * invMass);
		Matrix33d mat33_tmp(m_globalInertia);
		mat33_tmp *= (1.0 / dt + param.getAngularDamping());
		mat33_tmp += Matrix33d::Identity() * vtcParam.getVtcAngularDamping();
		Matrix33d matt33_inv = mat33_tmp.inverse();
		w  = matt33_inv * m_torque;
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
		qNorm = q.norm();
		// Normalize the quaternion to make sure we do have a rotation
		q.normalize();
	}
	m_currentState.setPose(SurgSim::Math::makeRigidTransform(q, G));

	// Compute the global inertia matrix with the current state
	updateGlobalInertiaMatrices(m_currentState);

	// If something went wrong, we deactivate the representation
	bool condition = SurgSim::Math::isValid(G);
	condition &= qNorm!=0.0;
	condition &= SurgSim::Math::isValid(q);
	condition &= fabs(1.0 - q.norm()) < 1e-3;
	SURGSIM_LOG_IF(! condition, Logger::getDefaultLogger(), DEBUG) << getName() <<
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

void VtcRigidRepresentation::afterUpdate(double dt)
{
}

void VtcRigidRepresentation::computeComplianceMatrix(double dt)
{
	if (! isActive() || ! m_currentParameters.isValid())
	{
		return;
	}

	m_C.setZero();

	//{ Id33.m.v(t+dt).[1/dt + alphaLinear  + alphaLinearVtc/m    + dt.k_t/m] =
	//                               = f                 + alphaLinearVtc.v(target)  + k_t.[x(target)-x(t)] + m.v(t)/dt
	//{ I     .w(t+dt).[1/dt + alphaAngular + I^-1.alphaAngularVtc          ] =
	//                               = t - w(t)^(I.w(t)) + alphaAngularVtc.w(target) + k_r.(alpha.u)        + I.w(t)/dt

	// Compliance matrix for interactions:
	// C = ( Mlinear^-1       0     )
	//     (   0        Mangular^-1 )
	// with Mlinear^-1  = Id33.(1/m)/(1/dt + alphaLinear  + alphaLinearVtc/m    + dt.k_t/m)
	// with Mangular^-1 = [I.(1/dt + alphaAngular) + Id33.alphaAngularVtc]^-1
	const double invMass = 1.0 / m_currentParameters.getMass();
	const double linDamp = m_currentParameters.getLinearDamping();
	const double angDamp = m_currentParameters.getAngularDamping();
	const double vtcLinDamp = m_currentVtcParameters.getVtcLinearDamping();
	const double vtcLinStif = m_currentVtcParameters.getVtcLinearStiffness();
	const double vtcAngDamp = m_currentVtcParameters.getVtcAngularDamping();

	double coefLin = 1.0/dt + linDamp + (vtcLinDamp + dt * vtcLinStif) * invMass;
	m_C(0,0) = m_C(1,1) = m_C(2,2) = invMass / coefLin;

	SurgSim::Math::Matrix33d mat33;
	const SurgSim::Math::Matrix33d id33 = SurgSim::Math::Matrix33d::Identity();
	mat33 = m_globalInertia * (1.0 / dt + angDamp) + id33 * vtcAngDamp;
	m_C.block<3,3>(3,3) = mat33.inverse();
}

void VtcRigidRepresentation::updateGlobalInertiaMatrices(const RigidRepresentationState& state)
{
	const SurgSim::Math::Matrix33d& R = state.getPose().rotation();
	m_globalInertia = R * m_currentParameters.getLocalInertia() * R.transpose();
	m_invGlobalInertia = m_globalInertia.inverse();
}


}; /// Physics

}; /// SurgSim
