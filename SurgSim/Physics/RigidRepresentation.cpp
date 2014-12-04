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

#include "SurgSim/Physics/RigidRepresentation.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidRepresentationState.h"

namespace
{
const double rotationVectorEpsilon = 1e-8;
};

namespace SurgSim
{
namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::RigidRepresentation, RigidRepresentation);

RigidRepresentation::RigidRepresentation(const std::string& name) :
	RigidRepresentationBase(name),
	m_globalInertia(SurgSim::Math::Matrix33d::Zero()),
	m_invGlobalInertia(SurgSim::Math::Matrix33d::Zero()),
	m_force(SurgSim::Math::Vector3d::Zero()),
	m_torque(SurgSim::Math::Vector3d::Zero()),
	m_C(SurgSim::Math::Matrix66d::Zero()),
	m_hasExternalGeneralizedForce(false),
	m_externalGeneralizedForce(SurgSim::Math::Vector6d::Zero()),
	m_externalGeneralizedStiffness(SurgSim::Math::Matrix66d::Zero()),
	m_externalGeneralizedDamping(SurgSim::Math::Matrix66d::Zero())
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

void RigidRepresentation::addExternalGeneralizedForce(const SurgSim::Math::Vector6d& generalizedForce,
													  const SurgSim::Math::Matrix66d& K,
													  const SurgSim::Math::Matrix66d& D)
{
	m_externalGeneralizedForce += generalizedForce;
	m_externalGeneralizedStiffness += K;
	m_externalGeneralizedDamping += D;
	m_hasExternalGeneralizedForce = true;
}

void RigidRepresentation::addExternalGeneralizedForce(const SurgSim::DataStructures::Location& location,
													  const SurgSim::Math::Vector6d& generalizedForce,
													  const SurgSim::Math::Matrix66d& K,
													  const SurgSim::Math::Matrix66d& D)
{
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Matrix66d;
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Vector6d;

	SURGSIM_ASSERT(location.rigidLocalPosition.hasValue()) << "Invalid location (no rigid local position)";

	RigidRepresentationLocalization localization;
	localization.setRepresentation(std::static_pointer_cast<Representation>(shared_from_this()));
	localization.setLocalPosition(location.rigidLocalPosition.getValue());
	const Vector3d point = localization.calculatePosition();
	const Vector3d massCenter = getCurrentState().getPose() * getMassCenter();
	const Vector3d lever = point - massCenter;
	auto force = generalizedForce.segment<3>(0);
	const Vector3d torque = lever.cross(force);

	// add the generalized force
	m_externalGeneralizedForce += generalizedForce;
	// add the generalized stiffness matrix
	m_externalGeneralizedStiffness += K;
	// add the generalized damping matrix
	m_externalGeneralizedDamping += D;

	// add the extra torque produced by the lever
	m_externalGeneralizedForce.segment<3>(3) += torque;

	// add the extra terms in the stiffness matrix
	const Vector3d leverInLocalSpace = getCurrentState().getPose().rotation().inverse() * lever;
	const Eigen::AngleAxisd angleAxis(getCurrentState().getPose().rotation());
	double angle = angleAxis.angle();
	const Vector3d axis = angleAxis.axis();
	const Vector3d rotationVector = axis * angle;
	double rotationVectorNorm = rotationVector.norm();
	double rotationVectorNormCubic = rotationVectorNorm * rotationVectorNorm * rotationVectorNorm;
	double sinAngle = sin(angle);
	double cosAngle = cos(angle);
	double oneMinusCos = 1.0 - cosAngle;
	const Matrix33d skewAxis = SurgSim::Math::makeSkewSymmetricMatrix(axis);
	Matrix33d dRdAxisX;
	Matrix33d dRdAxisY;
	Matrix33d dRdAxisZ;
	Matrix33d dRdAngle =
		-sinAngle * Matrix33d::Identity() + cosAngle * skewAxis + sinAngle * axis * axis.transpose();
	dRdAxisX << oneMinusCos * 2.0 * axis[0], oneMinusCos * axis[1], oneMinusCos * axis[2],
				oneMinusCos * axis[1], 0.0, -sinAngle,
				oneMinusCos * axis[2], sinAngle, 0.0;
	dRdAxisY << 0.0, oneMinusCos * axis[0], sinAngle,
				oneMinusCos * axis[0], oneMinusCos * 2.0 * axis[1], oneMinusCos * axis[2],
				-sinAngle, oneMinusCos * axis[2], 0.0;
	dRdAxisZ << 0.0, -sinAngle, oneMinusCos * axis[0],
				sinAngle, 0.0, oneMinusCos * axis[1],
				oneMinusCos * axis[0], oneMinusCos * axis[1], oneMinusCos * 2.0 * axis[2];
	Vector3d dAngledRotationVector, dAxisXdRotationVector, dAxisYdRotationVector, dAxisZdRotationVector;
	if (std::abs(rotationVectorNorm) > rotationVectorEpsilon)
	{
		const Vector3d tmp = rotationVector / rotationVectorNormCubic;
		dAngledRotationVector = rotationVector / rotationVectorNorm;
		dAxisXdRotationVector = Vector3d::UnitX() / rotationVectorNorm - rotationVector[0] * tmp;
		dAxisYdRotationVector = Vector3d::UnitY() / rotationVectorNorm - rotationVector[1] * tmp;
		dAxisZdRotationVector = Vector3d::UnitZ() / rotationVectorNorm - rotationVector[2] * tmp;
	}
	else
	{
		// Development around theta ~ 0 => sin(theta) ~ theta
		// We simplify by theta across the products dRdAxis[alpha].dAxis[alpha]dRotationVector[beta]
		dAngledRotationVector = Vector3d::Zero();
		dAxisXdRotationVector = Vector3d::UnitX();
		dAxisYdRotationVector = Vector3d::UnitY();
		dAxisZdRotationVector = Vector3d::UnitZ();

		dRdAxisX << 0.0, 0.0, 0.0,
					0.0, 0.0, -1.0,
					0.0, 1.0, 0.0;

		dRdAxisY << 0.0, 0.0, 1.0,
					0.0, 0.0, 0.0,
					-1.0, 0.0, 0.0;

		dRdAxisZ << 0.0, -1.0, 0.0,
					1.0, 0.0, 0.0,
					0.0, 0.0, 0.0;
	}
	// add the extra stiffness terms produced by the lever
	for (size_t column = 0; column < 6; ++column)
	{
		m_externalGeneralizedStiffness.block<3, 1>(3, column) += lever.cross(K.block<3, 1>(0, column));
	}
	// add extra term - dCP/dW[alpha] ^ F = - dR.CP(local)/dW[alpha] ^ F = -dR/dW[alpha].CP(local) ^ F
	// = -[(dR/dangle.dangle/dW[alpha] + dR/daxisX.daxisX/dW[alpha] +
	//      dR/daxisY.daxisY/dW[alpha] + dR/daxisZ.daxisZ/dW[alpha]) . CP(local)] ^ F
	for (size_t i = 0; i < 3; ++i)
	{
		m_externalGeneralizedStiffness.block<3, 1>(3, 3 + i) +=
			-((dRdAngle * dAngledRotationVector[i] +
			dRdAxisX * dAxisXdRotationVector[i] +
			dRdAxisY * dAxisYdRotationVector[i] +
			dRdAxisZ * dAxisZdRotationVector[i]) * leverInLocalSpace).cross(force);
	}

	// add the extra damping terms produced by the lever
	for (size_t column = 0; column < 6; ++column)
	{
		m_externalGeneralizedDamping.block<3, 1>(3, column) += lever.cross(D.block<3, 1>(0, column));
	}

	m_hasExternalGeneralizedForce = true;
}

const SurgSim::Math::Vector6d& RigidRepresentation::getExternalGeneralizedForce() const
{
	return m_externalGeneralizedForce;
}

const SurgSim::Math::Matrix66d& RigidRepresentation::getExternalGeneralizedStiffness() const
{
	return m_externalGeneralizedStiffness;
}

const SurgSim::Math::Matrix66d& RigidRepresentation::getExternalGeneralizedDamping() const
{
	return m_externalGeneralizedDamping;
}

void RigidRepresentation::beforeUpdate(double dt)
{
	RigidRepresentationBase::beforeUpdate(dt);

	SURGSIM_LOG_IF(!m_parametersValid,
				   SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
						   " deactivated in beforeUpdate because parameters are not valid." << std::endl;
	if (!m_parametersValid)
	{
		setLocalActive(false);
		return;
	}
}

void RigidRepresentation::update(double dt)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	SURGSIM_LOG_IF(!m_parametersValid,
				   SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
						   " deactivated in update because parameters are not valid." << std::endl;
	if (!m_parametersValid)
	{
		setLocalActive(false);
		return;
	}

	// For convenience
	Vector3d         G = m_currentState.getPose() * getMassCenter();
	Vector3d        dG = m_currentState.getLinearVelocity();
	Matrix33d        R = m_currentState.getPose().linear();
	Quaterniond      q = Quaterniond(R);
	Vector3d         w = m_currentState.getAngularVelocity();
	Quaterniond     dq;
	double       qNorm = q.norm(); // Norm of q before normalization

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
	if (m_hasExternalGeneralizedForce)
	{
		m_force += m_externalGeneralizedForce.segment<3>(0);
		m_torque += m_externalGeneralizedForce.segment<3>(3);
	}
	if (isGravityEnabled())
	{
		m_force += getGravity() * getMass();
	}
	m_torque -= w.cross(m_globalInertia * w);

	// Add Backward Euler RHS terms
	m_force  += getMass()     * dG / dt;
	m_torque += m_globalInertia *  w / dt;

	// Solve the 6D system on the velocity level
	{
		// { Id33.m.(1/dt + alphaLinear ).v(t+dt) = Id33.m.v(t)/dt + f
		// { I     .(1/dt + alphaAngular).w(t+dt) = I.w(t)/dt + t - w(t)^(I.w(t))
		dG = (1.0 / getMass()) * m_force  / (1.0 / dt + getLinearDamping());
		w  = m_invGlobalInertia * m_torque / (1.0 / dt + getAngularDamping());
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
	R = q.matrix();
	m_currentState.setPose(SurgSim::Math::makeRigidTransform(R, G-R*getMassCenter()));

	// Compute the global inertia matrix with the current state
	updateGlobalInertiaMatrices(m_currentState);

	// If something went wrong, we deactivate the representation
	bool condition = SurgSim::Math::isValid(G);
	condition &= SurgSim::Math::isValid(dG);
	condition &= SurgSim::Math::isValid(w);
	condition &= qNorm != 0.0;
	condition &= SurgSim::Math::isValid(q);
	condition &= fabs(1.0 - q.norm()) < 1e-3;
	SURGSIM_LOG_IF(!condition, SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
			" deactivated and reset because:" << std::endl <<
			"G=(" << G[0] << "," << G[1] << "," << G[2] << "), " <<
			"dG=(" << dG[0] << "," << dG[1] << "," << dG[2] << "), " <<
			"w=(" << w[0] << "," << w[1] << "," << w[2] << "), " <<
			"q=(" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "), " <<
			"|q| before normalization=" << qNorm << ", and " <<
			"|q| after normalization=" << q.norm() << std::endl;
	if (!condition)
	{
		setLocalActive(false);
	}

	// Prepare the compliance matrix
	computeComplianceMatrix(dt);
}

void RigidRepresentation::afterUpdate(double dt)
{
	RigidRepresentationBase::afterUpdate(dt);

	SURGSIM_LOG_IF(!m_parametersValid,
				   SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
						   " deactivated in afterUpdate because parameters are not valid." << std::endl;
	if (!m_parametersValid)
	{
		setLocalActive(false);
		return;
	}

	if (m_hasExternalGeneralizedForce)
	{
		m_hasExternalGeneralizedForce = false;
		m_externalGeneralizedForce.setZero();
		m_externalGeneralizedStiffness.setZero();
		m_externalGeneralizedDamping.setZero();
	}
}

void RigidRepresentation::applyCorrection(double dt,
										  const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	SURGSIM_LOG_IF(!m_parametersValid,
				   SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
						   " deactivated in applyCorrection because parameters are not valid." << std::endl;
	if (!m_parametersValid)
	{
		setLocalActive(false);
		return;
	}

	Vector3d          G = m_currentState.getPose() * getMassCenter();
	Vector3d         dG = m_currentState.getLinearVelocity();
	Matrix33d         R = m_currentState.getPose().linear();
	Quaterniond       q = Quaterniond(R);
	Vector3d          w = m_currentState.getAngularVelocity();

	const Vector3d& delta_dG = deltaVelocity.segment(0, 3);
	const Vector3d& delta_w  = deltaVelocity.segment(3, 3);
	Quaterniond delta_dq = Quaterniond(0.0, delta_w[0], delta_w[1], delta_w[2]) * q;
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
	R = q.matrix();
	m_currentState.setPose(SurgSim::Math::makeRigidTransform(R, G-R*getMassCenter()));

	// Compute the global inertia matrix with the current state
	updateGlobalInertiaMatrices(m_currentState);

	// If something went wrong, we deactivate the representation
	bool condition = SurgSim::Math::isValid(G);
	condition &= SurgSim::Math::isValid(q);
	condition &= fabs(1.0 - q.norm()) < 1e-3;
	SURGSIM_LOG_IF(!condition, SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
			" deactivated and reset in applyCorrection because:" << std::endl <<
			"G=(" << G[0] << "," << G[1] << "," << G[2] << "), " <<
			"q=(" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "), " <<
			"and |q| after normalization=" << q.norm() << std::endl;
	if (!condition)
	{
		setLocalActive(false);
	}

	// Prepare the compliance matrix
	computeComplianceMatrix(dt);
}

const Eigen::Matrix < double, 6, 6, Eigen::RowMajor > &
SurgSim::Physics::RigidRepresentation::getComplianceMatrix() const
{
	return m_C;
}

void RigidRepresentation::computeComplianceMatrix(double dt)
{
	SURGSIM_LOG_IF(!m_parametersValid,
				   SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << getName() <<
						   " deactivated in computComplianceMatrix because parameters are not valid." <<
						   std::endl;
	if (!m_parametersValid)
	{
		setLocalActive(false);
		return;
	}

	SurgSim::Math::Matrix66d systemMatrix;
	const SurgSim::Math::Matrix33d identity3x3 = SurgSim::Math::Matrix33d::Identity();
	systemMatrix.block<3, 3>(0, 0) = identity3x3 * (getMass() / dt + getLinearDamping());
	systemMatrix.block<3, 3>(3, 3) = m_globalInertia / dt + getAngularDamping() * identity3x3;
	if (m_hasExternalGeneralizedForce)
	{
		systemMatrix += m_externalGeneralizedDamping + m_externalGeneralizedStiffness * dt;
	}

	m_C.setZero();

	//Invert systemMatrix
	//We can use this shortcut because we know the linear and angular terms are independent
	m_C.block<3, 3>(0, 0) = systemMatrix.block<3, 3>(0, 0).inverse();
	m_C.block<3, 3>(3, 3) = systemMatrix.block<3, 3>(3, 3).inverse();
}

void RigidRepresentation::updateGlobalInertiaMatrices(const RigidRepresentationState& state)
{
	if (!m_parametersValid)
	{
		// do not setIsActive(false) due to invalid parameters because RigidRepresentationBase::setInitialParameters may
		// not have been called before RigidRepresentationBase::setInitialState (which calls this function), in which
		// case the parameters are invalid but we should not deactivate the representation.
		return;
	}

	const SurgSim::Math::Matrix33d& R = state.getPose().linear();
	m_globalInertia =  R * getLocalInertia() * R.transpose();
	m_invGlobalInertia = m_globalInertia.inverse();
}

bool RigidRepresentation::doInitialize()
{
	bool result = RigidRepresentationBase::doInitialize();

	if (result)
	{
		SURGSIM_ASSERT(getShape()->getVolume() > 0.0) << "Cannot use a shape with zero volume for RigidRepresentations";
	}

	return result;
}

void RigidRepresentation::setLinearVelocity(const SurgSim::Math::Vector3d& linearVelocity)
{
	m_currentState.setLinearVelocity(linearVelocity);
}

void RigidRepresentation::setAngularVelocity(const SurgSim::Math::Vector3d& angularVelocity)
{
	m_currentState.setAngularVelocity(angularVelocity);
}

}; // Physics
}; // SurgSim
