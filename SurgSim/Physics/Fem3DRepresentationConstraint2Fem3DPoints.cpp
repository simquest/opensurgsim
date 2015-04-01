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

#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/ConstraintDataFem3DDistancePoints.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationConstraint2Fem3DPoints.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Localization.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

Fem3DRepresentationConstraint2Fem3DPoints::Fem3DRepresentationConstraint2Fem3DPoints() :
	ConstraintImplementation()
{
}

Fem3DRepresentationConstraint2Fem3DPoints::~Fem3DRepresentationConstraint2Fem3DPoints()
{
}

void Fem3DRepresentationConstraint2Fem3DPoints::doBuild(double dt,
	const ConstraintData& data,
	const std::shared_ptr<Localization>& localization,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation,
	size_t indexOfConstraint,
	ConstraintSideSign sign)
{
	if (sign == CONSTRAINT_NEGATIVE_SIDE)
	{
		return;
	}

	std::shared_ptr<Fem3DRepresentation> fem3d
		= std::static_pointer_cast<Fem3DRepresentation>(localization->getRepresentation());

	if (!fem3d->isActive())
	{
		return;
	}

	const ConstraintDataFem3DDistancePoints *constraintData =
		static_cast<const ConstraintDataFem3DDistancePoints *>(&data);

	std::shared_ptr<Fem3DRepresentationLocalization> localization0 = constraintData->getPoint(0);
	std::shared_ptr<Fem3DRepresentationLocalization> localization1 = constraintData->getPoint(1);

	const SurgSim::DataStructures::IndexedLocalCoordinate& coord0 = localization0->getLocalPosition();
	const SurgSim::DataStructures::IndexedLocalCoordinate& coord1 = localization1->getLocalPosition();

	std::shared_ptr<FemElement> femElement0 = fem3d->getFemElement(coord0.index);
	std::shared_ptr<FemElement> femElement1 = fem3d->getFemElement(coord1.index);

	Vector3d P0 = localization0->calculatePosition();
	Vector3d P1 = localization1->calculatePosition();

	Vector3d P1P0 = P0 - P1;
	double distance = P1P0.norm();
	double requestedDistance = constraintData->getDistance();

	// Constraint = (P0-P1)^2 - d^2
	mlcp->b[indexOfConstraint] += distance * distance - requestedDistance * requestedDistance;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(fem3d->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	size_t numNodeToConstrain = (coord0.coordinate.size() + coord1.coordinate.size());
	m_newH.reserve(3 * numNodeToConstrain);
	m_newH.setZero();

	std::vector<Vector3d> Ai, Bi; ///< FemElements nodes for element 0 and 1
	for (size_t index = 0; index < femElement0->getNumNodes(); index++)
	{
		Ai.push_back(fem3d->getCurrentState()->getPosition(femElement0->getNodeId(index)));
	}
	for (size_t index = 0; index < femElement1->getNumNodes(); index++)
	{
		Bi.push_back(fem3d->getCurrentState()->getPosition(femElement1->getNodeId(index)));
	}

	// P0 = sum alpha_i Ai
	// P1 = sum beta_i Bi

	// d(C)/d(Ai)
	// d((P0 - P1)^2 - d^2)/d(Ai)
	// d(P0^2 + P1^2 - 2.P0.P1)/d(Ai)
	// d(P0^2)/d(Ai) - 2.d(P0.P1)/d(Ai)
	//   d(P0.P1)/d(Ai[axisI]) = d(sum(axis) P0[axis].P1[axis])/d(Ai[axisI]) = d(P0[axisI]P1[axisI])/d(Ai[axisI]) =
	//   d( (sum alpha_i Ai[axisI]) (sum beta_i Bi[axisI]) ) / d(Ai[axisI])
	//   alpha_i.P1[axisI]
	// 2.d(P0)/d(Ai).P0 - 2.alpha_i.P1
	// 2.alpha_i.P0 - 2.alpha_i.P1
	// 2.alpha_i (P0 - P1)
	for (size_t index = 0; index < femElement0->getNumNodes(); index++)
	{
		size_t nodeIndex = femElement0->getNodeId(index);
		for (size_t axis = 0; axis < 3; axis++)
		{
			m_newH.insert(3 * nodeIndex + axis) = dt * 2.0 * coord0.coordinate[axis] * P1P0[axis];
		}
	}

	// d(C)/d(Bi)
	// d((P0 - P1)^2 - d^2)/d(Bi)
	// d(P0^2 + P1^2 - 2.P0.P1)/d(Bi)
	// d(P1^2)/d(Bi) - 2.d(P0.P1)/d(Bi)
	//   d(P0.P1)/d(Bi[axisI]) = d(sum(axis) P0[axis].P1[axis])/d(Bi[axisI]) = d(P0[axisI]P1[axisI])/d(Bi[axisI]) =
	//   d( (sum alpha_i Ai[axisI]) (sum beta_i Bi[axisI]) ) / d(Bi[axisI])
	//   beta_i.P0[axisI]
	// 2.d(P1)/d(Bi).P1 - 2.beta_i.P0
	// 2.beta_i.P1 - 2.beta_i.P0
	// 2.beta_i (P1 - P0)
	for (size_t index = 0; index < femElement1->getNumNodes(); index++)
	{
		size_t nodeIndex = femElement1->getNodeId(index);
		for (size_t axis = 0; axis < 3; axis++)
		{
			m_newH.insert(3 * nodeIndex + axis) = -dt * 2.0 * coord1.coordinate[axis] * P1P0[axis];
		}
	}

	mlcp->updateConstraint(m_newH, fem3d->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint);
}

SurgSim::Math::MlcpConstraintType Fem3DRepresentationConstraint2Fem3DPoints::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_1D_CONSTRAINT;
}

size_t Fem3DRepresentationConstraint2Fem3DPoints::doGetNumDof() const
{
	return 1;
}

}; //  namespace Physics

}; //  namespace SurgSim
