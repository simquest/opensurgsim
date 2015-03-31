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

#include "SurgSim/Math/Valid.h"

#include "SurgSim/Physics/ConstraintDataRotationVector.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationConstraintRotationVector.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

RigidRepresentationConstraintRotationVector::RigidRepresentationConstraintRotationVector()
{
}

RigidRepresentationConstraintRotationVector::~RigidRepresentationConstraintRotationVector()
{
}

void RigidRepresentationConstraintRotationVector::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 size_t indexOfRepresentation,
											 size_t indexOfConstraint,
											 ConstraintSideSign sign)
{
	const double epsilon = 1e-7;

	std::shared_ptr<RigidRepresentation> rigid
		= std::static_pointer_cast<RigidRepresentation>(localization->getRepresentation());

	if (!rigid->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;

	Vector3d axis;
	double angle;
	SurgSim::Math::computeAngleAndAxis(SurgSim::Math::Quaterniond(rigid->getCurrentState().getPose().linear()), &angle, &axis);
	Vector3d currentRotationVector = angle * axis;

	// Zupan, E., Saje, M. 2011. Integrating rotation from angular velocity. Advances in
	// Engineering Software 42: 723-733, DOI: 10.1016/j.advengsoft.2011.05.010.
	//
	// Matrix(rotVector) = I + sin(angle)/angle.[rotVector] + (1-cos(angle))/angle^2.[rotVector]^2
	// d(rotVector)/dt = T^{-1}(rotVector).w   with w the angular velocity
	
	// Using an implicit integration scheme, we can calculate:
	// rotVector(t+dt) = rotVector(t) + dt.d(rotVector)/dt(t+dt)
	// rotVector(t+dt) = rotVector(t) + dt.T^{-1}(rotVector(t+dt)).w(t+dt)
	//   we approximate T^{-1}(rotVector(t+dt)) at time t, i.e. T^{-1}(rotVector(t))
	// rotVector(t+dt) = rotVector(t) + dt.T^{-1}(rotVector(t)).w(t+dt)

	// Fill up b with the constraint violation
	mlcp->b.segment<3>(indexOfConstraint) += currentRotationVector * scale;

	SurgSim::Math::Matrix33d TinvOfRotVector, skew;
	skew = SurgSim::Math::makeSkewSymmetricMatrix(currentRotationVector);
	
	if (std::abs(angle) < epsilon)
	{
		SurgSim::Math::Matrix33d TOfRotVector;
		TOfRotVector = SurgSim::Math::Matrix33d::Identity();
		TOfRotVector += 1.0 / 2.0 * skew;
		TOfRotVector += 1.0 / 6.0 * skew * skew;
		TinvOfRotVector = TOfRotVector.inverse();
	}
	else
	{
		TinvOfRotVector = SurgSim::Math::Matrix33d::Identity();
		TinvOfRotVector -= 0.5 * skew;
		TinvOfRotVector += (1.0 - angle / (2.0 * std::tan(angle / 2.0))) / (angle * angle) * skew * skew;
	}

	SURGSIM_ASSERT(SurgSim::Math::isValid(TinvOfRotVector)) << "Tinv(rotationVector) = " << TinvOfRotVector;

	// Fill up H with the transform from rigid body velocity -> constraint space
	m_newH.resize(rigid->getNumDof());
	m_newH.reserve(3);

	m_newH.insert(3 + 0) = dt * scale * TinvOfRotVector(0, 0);
	m_newH.insert(3 + 1) = dt * scale * TinvOfRotVector(0, 1);
	m_newH.insert(3 + 2) = dt * scale * TinvOfRotVector(0, 2);
	mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + 0);

	m_newH.setZero();
	m_newH.insert(3 + 0) = dt * scale * TinvOfRotVector(1, 0);
	m_newH.insert(3 + 1) = dt * scale * TinvOfRotVector(1, 1);
	m_newH.insert(3 + 2) = dt * scale * TinvOfRotVector(1, 2);
	mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + 1);

	m_newH.setZero();
	m_newH.insert(3 + 0) = dt * scale * TinvOfRotVector(2, 0);
	m_newH.insert(3 + 1) = dt * scale * TinvOfRotVector(2, 1);
	m_newH.insert(3 + 2) = dt * scale * TinvOfRotVector(2, 2);
	mlcp->updateConstraint(m_newH, rigid->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + 2);
}

SurgSim::Math::MlcpConstraintType RigidRepresentationConstraintRotationVector::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT;
}

size_t RigidRepresentationConstraintRotationVector::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim
