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

#include "SurgSim/Physics/FemConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemLocalization.h"
#include "SurgSim/Physics/FemRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

FemConstraintFixedRotationVector::FemConstraintFixedRotationVector()
{
}

FemConstraintFixedRotationVector::~FemConstraintFixedRotationVector()
{
}

void FemConstraintFixedRotationVector::doBuild(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	std::shared_ptr<FemRepresentation> fem
		= std::static_pointer_cast<FemRepresentation>(localization->getRepresentation());
	const size_t numDofPerNode = fem->getNumDofPerNode();

	SURGSIM_ASSERT(numDofPerNode >= 6) << "The rotation vector constraint needs rotational dof (6DOF/node minimum)"
		<< " but " << fem->getFullName() << " only has " << numDofPerNode << " DOF";

	if (!fem->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SurgSim::DataStructures::IndexedLocalCoordinate& coord
		= std::static_pointer_cast<FemLocalization>(localization)->getLocalPosition();

	auto nodeIds = fem->getFemElement(coord.index)->getNodeIds();
	Vector3d currentRotationVector = Vector3d::Zero();
	for (size_t i = 0; i < nodeIds.size(); i++)
	{
		currentRotationVector +=
			fem->getCurrentState()->getPositions().segment<3>(6 * nodeIds[i] + 3) * coord.coordinate[i];
	}

	// Fixed rotation vector constraint in MCLP
	//   p(t) is defined as the rotation vector before motion
	//   s is defined as the rotation vector to be constrained to
	//   u is defined as the displacement needed to enforce the constraint
	//
	// The equation is
	//   u + (p(t) - s) = 0
	//
	// Using backward-Euler integration,
	//   u = dt.v(t + dt)
	//
	// The constraint (p(t) - s) exists in 3-space, but we must modify the velocity of coordinates in (n * 3) space. The
	// transform from (n * 3) velocity space -> 3 translational space is denoted by H, which we construct here.
	//
	// The constructing principal of FEM is that nodes must be placed close enough such that the value of a function
	// within an FEM can be linearly interpolated by the values at the nodes of the FEM.  The interpolation weights are
	// given by barycentric coordinates which linearly transform the nodes from (n * 3) -> 3 space (and vice versa):
	//    sum(n_i * b_i) = n_1 * b_1 + n_2 * b_i ... n_n * b_n
	// where v_i are in 3 space.
	//
	// So the transform from node-velocity to constraint space is
	//    dt * sum(v_i * b_i)
	//
	// See RigidRepresentationFixedPoint for more implementation details.

	// Update b with new violation: P(free motion)
	mlcp->b.segment<3>(indexOfConstraint) += currentRotationVector * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(fem->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	size_t numNodeToConstrain = (coord.coordinate.array() != 0.0).count();
	m_newH.reserve(3 * numNodeToConstrain);

	std::shared_ptr<FemElement> femElement = fem->getFemElement(coord.index);
	size_t numNodes = femElement->getNumNodes();
	for (size_t axis = 0; axis < 3; axis++)
	{
		m_newH.setZero();
		for (size_t index = 0; index < numNodes; index++)
		{
			if (coord.coordinate[index] != 0.0)
			{
				size_t nodeIndex = femElement->getNodeId(index);
				m_newH.insert(numDofPerNode * nodeIndex + axis + 3) = coord.coordinate[index] * (dt * scale);
			}
		}
		mlcp->updateConstraint(m_newH, fem->getComplianceMatrix() * m_newH.transpose(),
			indexOfRepresentation, indexOfConstraint + axis);
	}
}

SurgSim::Physics::ConstraintType FemConstraintFixedRotationVector::getConstraintType() const
{
	return SurgSim::Physics::FIXED_3DROTATION_VECTOR;
}

size_t FemConstraintFixedRotationVector::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim
