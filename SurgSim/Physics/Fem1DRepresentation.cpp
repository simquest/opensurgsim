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
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

using SurgSim::Math::SparseMatrix;

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
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Fem1DRepresentation, std::shared_ptr<SurgSim::Framework::Asset>, Fem, getFem,
									  setFem);
	SURGSIM_ADD_SETTER(Fem1DRepresentation, std::string, FemFileName, loadFem)
	// Reminder: m_numDofPerNode is held by DeformableRepresentation but needs to be set by all
	// concrete derived classes
	m_numDofPerNode = 6;
	m_fem = std::make_shared<Fem1D>();
}

Fem1DRepresentation::~Fem1DRepresentation()
{
}

void Fem1DRepresentation::loadFem(const std::string& fileName)
{
	auto mesh = std::make_shared<Fem1D>();
	mesh->load(fileName);
	setFem(mesh);
}

void Fem1DRepresentation::setFem(std::shared_ptr<Framework::Asset> mesh)
{
	SURGSIM_ASSERT(!isInitialized()) << "The Fem cannot be set after initialization";

	SURGSIM_ASSERT(mesh != nullptr) << "Mesh for Fem1DRepresentation cannot be a nullptr";
	auto femMesh = std::dynamic_pointer_cast<Fem1D>(mesh);
	SURGSIM_ASSERT(femMesh != nullptr)
			<< "Mesh for Fem1DRepresentation needs to be a SurgSim::Physics::Fem1D";
	m_fem = femMesh;
	auto state = std::make_shared<SurgSim::Math::OdeState>();

	state->setNumDof(getNumDofPerNode(), m_fem->getNumVertices());
	for (size_t i = 0; i < m_fem->getNumVertices(); i++)
	{
		state->getPositions().segment<3>(getNumDofPerNode() * i) = m_fem->getVertexPosition(i);
	}
	for (auto boundaryCondition : m_fem->getBoundaryConditions())
	{
		state->addBoundaryCondition(boundaryCondition);
	}

	// If we have elements, ensure that they are all of the same nature
	if (femMesh->getNumElements() > 0)
	{
		auto e0 = femMesh->getElement(0);
		for (auto const& e : femMesh->getElements())
		{
			SURGSIM_ASSERT(e->nodeIds.size() == e0->nodeIds.size()) << "Cannot mismatch element of different nature";
		}

		// If the FemElement types hasn't been registered yet, let's set a default one
		if (getFemElementType().empty())
		{
			SURGSIM_ASSERT(e0->nodeIds.size() == 2) <<
				"Invalid Element size. Expected a beam, found a size " << e0->nodeIds.size();
			Fem1DElementBeam beam;
			setFemElementType(beam.getClassName());
		}
	}

	FemRepresentation::setInitialState(state);
}

std::shared_ptr<Fem1D> Fem1DRepresentation::getFem() const
{
	return m_fem;
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

	std::shared_ptr<Fem1DLocalization> localization1D =
			std::dynamic_pointer_cast<Fem1DLocalization>(localization);
	SURGSIM_ASSERT(localization1D != nullptr) << "Invalid localization type (not a Fem1DLocalization)";

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
					Math::addSubMatrix(coordinate[index1] * coordinate[index2] * K,
									   static_cast<SparseMatrix::Index>(nodeId1),
									   static_cast<SparseMatrix::Index>(nodeId2),
									   &m_externalGeneralizedStiffness, true);
				}
				if (D.size() != 0)
				{
					Math::addSubMatrix(coordinate[index1] * coordinate[index2] * D,
									   static_cast<SparseMatrix::Index>(nodeId1),
									   static_cast<SparseMatrix::Index>(nodeId2),
									   &m_externalGeneralizedDamping, true);
				}
				index2++;
			}

			index1++;
		}
	}
	m_hasExternalGeneralizedForce = true;
}

void Fem1DRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
										 const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

bool Fem1DRepresentation::doInitialize()
{
	for (auto& element : m_fem->getElements())
	{
		std::shared_ptr<FemElement> femElement;
		femElement = FemElement::getFactory().create(getFemElementType(), element);
		m_femElements.push_back(femElement);
	}

	return FemRepresentation::doInitialize();
}

} // namespace Physics

} // namespace SurgSim
