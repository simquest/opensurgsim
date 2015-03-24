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
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/ConstraintDataRotationVector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationConstraintRotationVector.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Localization.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

Fem3DRepresentationConstraintRotationVector::Fem3DRepresentationConstraintRotationVector()
{
}

Fem3DRepresentationConstraintRotationVector::~Fem3DRepresentationConstraintRotationVector()
{
}

// d (rotVec) / d (dofId)
Vector3d Fem3DRepresentationConstraintRotationVector::numericalEvaluationRotVectorJacobian(
	std::shared_ptr<Fem3DRepresentation> fem3d,
	FemElement* femElement,
	SurgSim::Math::Quaterniond originalStaplerRotation,
	SurgSim::Math::Quaterniond originalFemElementOrientation,
	SurgSim::Math::Quaterniond currentFemElementOrientation,
	size_t dofId, double epsilon)
{
	Vector3d axis;
	double angle;
	const size_t nDofPerNode = fem3d->getNumDofPerNode();

	if (m_tmpFem3DState == nullptr)
	{
		m_tmpFem3DState = std::make_shared<SurgSim::Math::OdeState>();
		m_tmpFem3DState->setNumDof(nDofPerNode, fem3d->getInitialState()->getNumNodes());
	}

	// We only care about the positions to determine the orientation of the element
	// And we only care about the nodes connected to this specific FemElement
	for (auto nodeId : femElement->getNodeIds())
	{
		m_tmpFem3DState->getPositions().segment(nodeId * nDofPerNode, nDofPerNode) =
			fem3d->getCurrentState()->getPositions().segment(nodeId * nDofPerNode, nDofPerNode);
	}

	Vector3d rotVec;
	// df/dx = (f(x) - f(x+h))/h
	{
		// Evaluate f(x)
		SurgSim::Math::Quaterniond  deltaRotation = (currentFemElementOrientation * originalFemElementOrientation.inverse()).normalized();

		SurgSim::Math::Quaterniond newStaplerOrientation = deltaRotation * originalStaplerRotation;
		SurgSim::Math::computeAngleAndAxis(newStaplerOrientation, &angle, &axis);
		rotVec = angle * axis;
	}

	Vector3d rotVecEpsilon;
	{
		// Evaluate f(x+h)
		m_tmpFem3DState->getPositions()[dofId] += epsilon;

		SurgSim::Math::Quaterniond currentFemElementOrientationEpsilon = femElement->getOrientation(*m_tmpFem3DState);
		SurgSim::Math::Quaterniond  deltaRotation = (currentFemElementOrientationEpsilon * originalFemElementOrientation.inverse()).normalized();

		SurgSim::Math::Quaterniond newStaplerOrientation = deltaRotation * originalStaplerRotation;
		SurgSim::Math::computeAngleAndAxis(newStaplerOrientation, &angle, &axis);
		rotVecEpsilon = angle * axis;
	}

	//std::cout << rotVec.transpose() << "  +eps=" << rotVecEpsilon.transpose() << "  +delta =" << ((rotVec - rotVecEpsilon) / epsilon).transpose() << std::endl;

	return ((rotVec - rotVecEpsilon) / epsilon).eval();
}

// rotVec
Vector3d Fem3DRepresentationConstraintRotationVector::numericalEvaluationRotVector(
	std::shared_ptr<Fem3DRepresentation> fem3d,
	FemElement* femElement,
	SurgSim::Math::Quaterniond originalStaplerRotation,
	SurgSim::Math::Quaterniond originalFemElementOrientation,
	SurgSim::Math::Quaterniond currentFemElementOrientation)
{
	SurgSim::Math::Quaterniond  deltaRotation = (currentFemElementOrientation * originalFemElementOrientation.inverse()).normalized();

	SurgSim::Math::Quaterniond newStaplerOrientation = deltaRotation * originalStaplerRotation;
	Vector3d axis;
	double angle;
	SurgSim::Math::computeAngleAndAxis(newStaplerOrientation, &angle, &axis);
	return angle * axis;
}

// Constrain the rotation vector of a specific location on the Fem3D
// rotVec(localization) = f(femElement.nodes)
// d (rotVec) / d (dof)
void Fem3DRepresentationConstraintRotationVector::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 size_t indexOfRepresentation,
											 size_t indexOfConstraint,
											 ConstraintSideSign sign)
{
	using namespace SurgSim::Framework;

	std::shared_ptr<Fem3DRepresentation> fem3d
		= std::static_pointer_cast<Fem3DRepresentation>(localization->getRepresentation());

	if (!fem3d->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE) ? 1.0 : -1.0;
	const SurgSim::DataStructures::IndexedLocalCoordinate& coord
		= std::static_pointer_cast<Fem3DRepresentationLocalization>(localization)->getLocalPosition();

	const ConstraintDataRotationVector* dataRotationVector = dynamic_cast<const ConstraintDataRotationVector*>(&data);
	SURGSIM_ASSERT(dataRotationVector != nullptr) << "Invalid constraint data type";

	std::shared_ptr<FemElement> femElement = fem3d->getFemElement(coord.index);

	SurgSim::Math::Quaterniond originalRigidOrientation = SurgSim::Math::makeRotationQuaternion(
		dataRotationVector->getInitialRotationVector().norm(), dataRotationVector->getInitialRotationVector().normalized());

	SurgSim::Math::Quaterniond originalOrientation = dataRotationVector->getInitialOrientation();
	SurgSim::Math::Quaterniond currentFemElementOrientation = femElement->getOrientation(*fem3d->getCurrentState());

	Vector3d rotationVector = numericalEvaluationRotVector(fem3d, femElement.get(), originalRigidOrientation,
		dataRotationVector->getInitialOrientation(), currentFemElementOrientation);

	SurgSim::Math::Matrix jacobian(3, femElement->getNumDofPerNode() * femElement->getNumNodes());
	size_t elementNodeId = 0;
	double epsilon = 1e-7;
	for (size_t nodeId : femElement->getNodeIds())
	{
		for (size_t axis = 0; axis < 3; axis++)
		{
			jacobian.col(elementNodeId * 3 + axis) = numericalEvaluationRotVectorJacobian(fem3d, femElement.get(), originalRigidOrientation,
				dataRotationVector->getInitialOrientation(), currentFemElementOrientation, nodeId * 3 + axis, epsilon);
		}
		elementNodeId++;
	}

	//std::cout << "rotVec = " << rotationVector.transpose() << std::endl;
	//std::cout << "d(rotVec)/d(dofs) = " << jacobian << std::endl;

	// Update b with new violation: P(free motion)
	//mlcp->b.segment<3>(indexOfConstraint) += dataRotationVector.getInitialRotationVector() * scale;
	mlcp->b.segment<3>(indexOfConstraint) +=  rotationVector * scale;

	// m_newH is a SparseVector, so resizing is cheap.  The object's memory also gets cleared.
	m_newH.resize(fem3d->getNumDof());
	// m_newH is a member variable, so 'reserve' only needs to allocate memory on the first run.
	size_t numNodeToConstrain = coord.coordinate.size();
	m_newH.reserve(3 * numNodeToConstrain);

	size_t numNodes = femElement->getNumNodes();
	for (size_t constraint = 0; constraint < 3; constraint++)
	{
		m_newH.setZero();
		for (size_t index = 0; index < numNodes; index++)
		{
			size_t nodeIndex = femElement->getNodeId(index);
			for (size_t axis = 0; axis < 3; axis++)
			{
				//if (coord.coordinate[index] != 0.0)
				{
					m_newH.insert(3 * nodeIndex + axis) =
						jacobian(constraint, 3 * index + axis) * scale * dt * coord.coordinate[index];
				}
			}
		}
		mlcp->updateConstraint(m_newH, fem3d->getComplianceMatrix(), indexOfRepresentation, indexOfConstraint + constraint);
	}
}

SurgSim::Math::MlcpConstraintType Fem3DRepresentationConstraintRotationVector::getMlcpConstraintType() const
{
	return SurgSim::Math::MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT;
}

size_t Fem3DRepresentationConstraintRotationVector::doGetNumDof() const
{
	return 3;
}

}; //  namespace Physics

}; //  namespace SurgSim
