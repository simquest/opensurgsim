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

#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationBilateral3D.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/Localization.h"

using SurgSim::Math::Vector3d;

static size_t countNonZero(Eigen::VectorXd vector)
{
	size_t count = 0;
	size_t size = vector.size();

	for (size_t i = 0; i < size; i++)
	{
		if (vector[i] != 0.0)
		{
			count++;
		}
	}

	return count;
}

namespace SurgSim
{

namespace Physics
{

Fem3DRepresentationBilateral3D::Fem3DRepresentationBilateral3D()
{
}

Fem3DRepresentationBilateral3D::~Fem3DRepresentationBilateral3D()
{
}

void Fem3DRepresentationBilateral3D::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 unsigned int indexOfRepresentation,
											 unsigned int indexOfConstraint,
											 ConstraintSideSign sign)
{
	std::shared_ptr<Fem3DRepresentation> fem3d
		= std::static_pointer_cast<Fem3DRepresentation>(localization->getRepresentation());

	if (!fem3d->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const FemRepresentationCoordinate& coord
		= std::static_pointer_cast<Fem3DRepresentationLocalization>(localization)->getLocalPosition();

	Vector3d globalPosition = localization->calculatePosition();

	// Bilateral3d in a LCP
	//   P(t+dt) = other side of the constraint...
	//   P(free motion) + dt.v(t+dt) = other side of the constraint...
	//   p(free) is the point of contact after free motion
	//   u is the displacement needed to verify the constraint
	// note that u = sum ui * baryCoord[i]
	//
	// The constraint equation is
	// P(free motion) + dt.v = ... (Using Backward Euler integration scheme: x(t+dt)-x(t) = dt.v(t+dt))

	// Update b with new violation: P(free motion)
	mlcp->b.segment<3>(indexOfConstraint) += globalPosition * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(fem3d->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	size_t numNodeToConstrain = countNonZero(coord.naturalCoordinate);
	m_newH.reserve(3 * numNodeToConstrain);

	std::shared_ptr<FemElement> femElement = fem3d->getFemElement(coord.elementId);
	size_t numNodes = femElement->getNumNodes();
	for (size_t axis = 0; axis < 3; axis++)
	{
		m_newH.setZero();
		for (size_t index = 0; index < numNodes; index++)
		{
			if (coord.naturalCoordinate[index] != 0.0)
			{
				unsigned int nodeIndex = femElement->getNodeId(index);
				m_newH.insert(3 * nodeIndex + axis) = coord.naturalCoordinate[index] * (dt * scale);
			}
		}
		mlcp->updateConstraint(m_newH, fem3d->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + axis);
	}
}

SurgSim::Math::MlcpConstraintType Fem3DRepresentationBilateral3D::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
}

SurgSim::Physics::RepresentationType Fem3DRepresentationBilateral3D::getRepresentationType() const
{
	return REPRESENTATION_TYPE_FEM3D;
}

unsigned int Fem3DRepresentationBilateral3D::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim
