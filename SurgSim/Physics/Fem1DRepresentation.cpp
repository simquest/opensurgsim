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

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/LinearSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentationLocalization.h"
#include "SurgSim/Physics/FemElement.h"

namespace
{

void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform, SurgSim::Math::Vector* x,
							   bool rotationOnly = false)
{
	typedef SurgSim::Math::Vector::Index IndexType;

	IndexType numNodes = x->size() / 6;

	SURGSIM_ASSERT(numNodes * 6 == x->size())
		<< "Unexpected number of dof in a Fem1D state vector (not a multiple of 6)";

	for (IndexType nodeId = 0; nodeId < numNodes; nodeId++)
	{
		// Only the translational dof are transformed, rotational dof remains unchanged
		SurgSim::Math::Vector3d xi = x->segment<3>(6 * nodeId);

		x->segment<3>(6 * nodeId) = (rotationOnly) ? transform.linear() * xi : transform * xi;
	}
}

}

namespace SurgSim
{

namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::Fem1DRepresentation, Fem1DRepresentation);

Fem1DRepresentation::Fem1DRepresentation(const std::string& name) : FemRepresentation(name)
{
	// Reminder: m_numDofPerNode is held by DeformableRepresentation but needs to be set by all concrete derived classes
	m_numDofPerNode = 6;
}

Fem1DRepresentation::~Fem1DRepresentation()
{
}

RepresentationType Fem1DRepresentation::getType() const
{
	return REPRESENTATION_TYPE_FEM1D;
}

void Fem1DRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
										 const SurgSim::Math::Vector& generalizedForce,
										 const SurgSim::Math::Matrix& K,
										 const SurgSim::Math::Matrix& D)
{
	const size_t dofPerNode = getNumDofPerNode();
	const SurgSim::Math::Matrix::Index expectedSize = static_cast<const SurgSim::Math::Matrix::Index>(dofPerNode);

	SURGSIM_ASSERT(localization != nullptr) << "Invalid localization (nullptr)";
	SURGSIM_ASSERT(generalizedForce.size() == expectedSize) <<
		"Generalized force has an invalid size of " << generalizedForce.size() << ". Expected " << dofPerNode;
	SURGSIM_ASSERT(K.size() == 0 || (K.rows() == expectedSize && K.cols() == expectedSize)) <<
		"Stiffness matrix K has an invalid size (" << K.rows() << "," << K.cols() <<
		") was expecting a square matrix of size " << dofPerNode;
	SURGSIM_ASSERT(D.size() == 0 || (D.rows() == expectedSize && D.cols() == expectedSize)) <<
		"Damping matrix D has an invalid size (" << D.rows() << "," << D.cols() <<
		") was expecting a square matrix of size " << dofPerNode;

	std::shared_ptr<Fem1DRepresentationLocalization> localization1D =
		std::dynamic_pointer_cast<Fem1DRepresentationLocalization>(localization);
	SURGSIM_ASSERT(localization1D != nullptr) << "Invalid localization type (not a Fem1DRepresentationLocalization)";

	const size_t elementId = localization1D->getLocalPosition().index;
	const SurgSim::Math::Vector& coordinate = localization1D->getLocalPosition().coordinate;
	std::shared_ptr<FemElement> element = getFemElement(elementId);

	size_t index = 0;
	for (auto nodeId : element->getNodeIds())
	{
		m_externalGeneralizedForce.segment(dofPerNode * nodeId, dofPerNode) += generalizedForce * coordinate[index];
		index++;
	}

	if (K.size() != 0 || D.size() != 0)
	{
		size_t index1 = 0;
		for (auto nodeId1 : element->getNodeIds())
		{
			size_t index2 = 0;
			for (auto nodeId2 : element->getNodeIds())
			{
				if (K.size() != 0)
				{
					m_externalGeneralizedStiffness.block(dofPerNode * nodeId1, dofPerNode * nodeId2,
						dofPerNode, dofPerNode) += coordinate[index1] * coordinate[index2] * K;
				}
				if (D.size() != 0)
				{
					m_externalGeneralizedDamping.block(dofPerNode * nodeId1, dofPerNode * nodeId2,
						dofPerNode, dofPerNode) += coordinate[index1] * coordinate[index2] * D;
				}
				index2++;
			}

			index1++;
		}
	}
	m_hasExternalGeneralizedForce = true;
}

bool Fem1DRepresentation::doWakeUp()
{
	using SurgSim::Math::LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix;

	if (!FemRepresentation::doWakeUp())
	{
		return false;
	}

	// Make use of a specialized linear solver for symmetric tri-diagonal block matrix of block size 6
	m_odeSolver->setLinearSolver(std::make_shared<LinearSolveAndInverseSymmetricTriDiagonalBlockMatrix<6>>());

	return true;
}

void Fem1DRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
										 const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

std::shared_ptr<FemPlyReaderDelegate> Fem1DRepresentation::getDelegate()
{
	auto thisAsSharedPtr = std::static_pointer_cast<Fem1DRepresentation>(shared_from_this());
	auto readerDelegate = std::make_shared<Fem1DPlyReaderDelegate>(thisAsSharedPtr);

	return readerDelegate;
}

} // namespace Physics

} // namespace SurgSim
