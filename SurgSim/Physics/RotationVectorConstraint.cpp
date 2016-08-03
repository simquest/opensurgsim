// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/RotationVectorConstraint.h"

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{

namespace Physics
{

RotationVectorConstraint::RotationVectorConstraint(ConstraintType constraintType,
	std::shared_ptr<ConstraintData> data,
	std::shared_ptr<Representation> representation0,
	const SurgSim::DataStructures::Location& location0,
	std::shared_ptr<Representation> representation1,
	const SurgSim::DataStructures::Location& location1)
	: Constraint(constraintType, data, representation0, location0, representation1, location1)
{
	SURGSIM_ASSERT(constraintType == SurgSim::Physics::ConstraintType::FIXED_3DROTATION_VECTOR) <<
		"Invalid constraint type for a rotation vector constraint";
}

RotationVectorConstraint::~RotationVectorConstraint()
{
}

void RotationVectorConstraint::doBuild(double dt,
	const ConstraintData& data,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation0,
	size_t indexOfRepresentation1,
	size_t indexOfConstraint)
{
	m_implementations.first->build(
		dt,
		*m_data.get(),
		m_localizations.first,
		mlcp,
		indexOfRepresentation0,
		indexOfConstraint,
		CONSTRAINT_POSITIVE_SIDE);

	SurgSim::Math::Vector3d rotationVector1 = mlcp->b.segment<3>(indexOfConstraint);
	SurgSim::Math::Quaterniond q1 = SurgSim::Math::Quaterniond::Identity();
	if (rotationVector1.norm() > 1e-8)
	{
		q1 = SurgSim::Math::makeRotationQuaternion(rotationVector1.norm(), rotationVector1.normalized());
	}

	m_implementations.second->build(
		dt,
		*m_data.get(),
		m_localizations.second,
		mlcp,
		indexOfRepresentation1,
		indexOfConstraint,
		CONSTRAINT_NEGATIVE_SIDE);

	SurgSim::Math::Vector3d rotationVector2 = rotationVector1 - mlcp->b.segment<3>(indexOfConstraint);
	SurgSim::Math::Quaterniond q2 = SurgSim::Math::Quaterniond::Identity();
	if (rotationVector2.norm() > 1e-8)
	{
		q2 = SurgSim::Math::makeRotationQuaternion(rotationVector2.norm(), rotationVector2.normalized());
	}

	// Transform the data to have a rotation vector violation as a rotation vector
	double angle;
	SurgSim::Math::Vector3d axis;
	SurgSim::Math::computeAngleAndAxis((q1 * q2.inverse()).normalized(), &angle, &axis);
	mlcp->b.segment<3>(indexOfConstraint) = angle * axis;

	mlcp->constraintTypes.push_back(
		(m_constraintType != INVALID_CONSTRAINT) ? m_mlcpMap[m_constraintType] : Math::MLCP_INVALID_CONSTRAINT);
}

}; // namespace Physics

}; // namespace SurgSim
