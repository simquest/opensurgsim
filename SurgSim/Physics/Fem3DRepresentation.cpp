// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Localization.h"

namespace
{
void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform, SurgSim::Math::Vector* x,
							   bool rotationOnly = false)
{
	typedef SurgSim::Math::Vector::Index IndexType;

	IndexType numNodes = x->size() / 3;
	SURGSIM_ASSERT(numNodes * 3 == x->size())
			<< "Unexpected number of dof in a Fem3D state vector (not a multiple of 3)";

	for (IndexType nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::Vector3d xi = x->segment<3>(3 * nodeId);
		x->segment<3>(3 * nodeId) = (rotationOnly) ? transform.linear() * xi : transform * xi;
	}
}
}

namespace SurgSim
{
namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::Fem3DRepresentation, Fem3DRepresentation);

Fem3DRepresentation::Fem3DRepresentation(const std::string& name) :
	FemRepresentation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Fem3DRepresentation, std::shared_ptr<SurgSim::Framework::Asset>, Fem, getFem,
									  setFem);
	SURGSIM_ADD_SETTER(Fem3DRepresentation, std::string, FemFileName, loadFem)
	// Reminder: m_numDofPerNode is held by DeformableRepresentation
	// but needs to be set by all concrete derived classes
	m_numDofPerNode = 3;
	m_fem = std::make_shared<Fem3D>();
}

Fem3DRepresentation::~Fem3DRepresentation()
{
}

void Fem3DRepresentation::loadFem(const std::string& fileName)
{
	auto mesh = std::make_shared<Fem3D>();
	mesh->load(fileName);
	setFem(mesh);
}

void Fem3DRepresentation::setFem(std::shared_ptr<Framework::Asset> mesh)
{
	SURGSIM_ASSERT(!isInitialized()) << "The Fem cannot be set after initialization";

	SURGSIM_ASSERT(mesh != nullptr) << "Mesh for Fem3DRepresentation cannot be a nullptr";
	auto femMesh = std::dynamic_pointer_cast<Fem3D>(mesh);
	SURGSIM_ASSERT(femMesh != nullptr)
			<< "Mesh for Fem3DRepresentation needs to be a SurgSim::Physics::Fem3D";
	m_fem = femMesh;
	auto state = std::make_shared<Math::OdeState>();

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
		const auto& e0 = femMesh->getElement(0);
		for (auto const& e : femMesh->getElements())
		{
			SURGSIM_ASSERT(e->nodeIds.size() == e0->nodeIds.size()) <<
				"Cannot mix and match elements of different nature." <<
				" Found an element with " << e->nodeIds.size() << " nodes but was expecting " << e0->nodeIds.size();
		}

		// If the FemElement types hasn't been registered yet, let's set a default one
		if (getFemElementType().empty())
		{
			if (e0->nodeIds.size() == 4)
			{
				Fem3DElementTetrahedron tetrahdron;
				setFemElementType(tetrahdron.getClassName());
			}
			else if (e0->nodeIds.size() == 8)
			{
				Fem3DElementCube cube;
				setFemElementType(cube.getClassName());
			}
		}
	}

	setInitialState(state);
}

std::shared_ptr<Fem3D> Fem3DRepresentation::getFem() const
{
	return m_fem;
}

void Fem3DRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
													  const Math::Vector& generalizedForce,
													  const Math::Matrix& K,
													  const Math::Matrix& D)
{
	using Math::SparseMatrix;

	const size_t dofPerNode = getNumDofPerNode();
	const Math::Matrix::Index expectedSize = static_cast<const Math::Matrix::Index>(dofPerNode);

	SURGSIM_ASSERT(localization != nullptr) << "Invalid localization (nullptr)";
	SURGSIM_ASSERT(generalizedForce.size() == expectedSize) <<
				"Generalized force has an invalid size of " << generalizedForce.size() << ". Expected " << dofPerNode;
	SURGSIM_ASSERT(K.size() == 0 || (K.rows() == expectedSize && K.cols() == expectedSize)) <<
					"Stiffness matrix K has an invalid size (" << K.rows() << "," << K.cols() <<
					") was expecting a square matrix of size " << dofPerNode;
	SURGSIM_ASSERT(D.size() == 0 || (D.rows() == expectedSize && D.cols() == expectedSize)) <<
					"Damping matrix D has an invalid size (" << D.rows() << "," << D.cols() <<
					") was expecting a square matrix of size " << dofPerNode;

	std::shared_ptr<Fem3DLocalization> localization3D =
			std::dynamic_pointer_cast<Fem3DLocalization>(localization);
	SURGSIM_ASSERT(localization3D != nullptr) << "Invalid localization type (not a Fem3DLocalization)";

	const size_t elementId = localization3D->getLocalPosition().index;
	const Math::Vector& coordinate = localization3D->getLocalPosition().coordinate;
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
	m_externalGeneralizedStiffness.makeCompressed();
	m_externalGeneralizedDamping.makeCompressed();
	m_hasExternalGeneralizedForce = true;
}

std::unordered_map<size_t, size_t> Fem3DRepresentation::createTriangleIdToElementIdMap(
		std::shared_ptr<const Math::MeshShape> mesh)
{
	std::unordered_map<size_t, size_t> result;

	// An Fem3DElementCube/Fem3DElementTetrahedron element has 8/4 nodes.
	// A triangle has 3 nodes.
	// If all the nodes of a triangle are present in a Fem3DElement*** node, then a row in the map is created.
	// The nodes are identified using their ids.
	// std::includes(...) is used to find whether a given list of triangle node ids are present in the supplied list
	// of femElement node ids. This function requires the lists of node ids to be sorted.

	// Get the list of fem elements with their node ids.
	std::vector<std::vector<size_t>> femElements;
	femElements.reserve(getNumFemElements());
	for (size_t i = 0; i < getNumFemElements(); ++i)
	{
		auto elementNodeIds = getFemElement(i)->getNodeIds();
		std::sort(elementNodeIds.begin(), elementNodeIds.end());
		femElements.push_back(elementNodeIds);
	}

	std::array<size_t, 3> triangleSorted;
	auto doesIncludeTriangle = [&triangleSorted](const std::vector<size_t>& femElementSorted)
	{
		return std::includes(femElementSorted.begin(), femElementSorted.end(),
							 triangleSorted.begin(), triangleSorted.end());
	};

	auto& meshTriangles = mesh->getTriangles();
	for (auto triangle = meshTriangles.cbegin(); triangle != meshTriangles.cend(); ++triangle)
	{
		if (! triangle->isValid)
		{
			continue;
		}
		triangleSorted = triangle->verticesId;
		std::sort(triangleSorted.begin(), triangleSorted.end());

		// Find the femElement that contains all the node ids of this triangle.
		std::vector<std::vector<size_t>>::iterator foundFemElement =
				std::find_if(femElements.begin(), femElements.end(), doesIncludeTriangle);

		// Assert to make sure that a triangle doesn't end up not having a femElement mapped to it.
		SURGSIM_ASSERT(foundFemElement != femElements.end())
				<< "A triangle in the given mesh of an Fem3DRepresentation does not have a corresponding"
				<< " femElement.";

		// Add a row to the mapping (triangleId, elementId).
		// std::distance gives the index of the iterator within the container (by finding the distance
		// from the beginning of the container).
		result[std::distance(meshTriangles.begin(), triangle)] = std::distance(femElements.begin(), foundFemElement);
	}

	return result;
}

bool Fem3DRepresentation::doWakeUp()
{
	if (!FemRepresentation::doWakeUp())
	{
		return false;
	}

	auto deformableCollision = std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);
	if (deformableCollision != nullptr)
	{
		auto mesh = std::dynamic_pointer_cast<Math::MeshShape>(deformableCollision->getShape());
		m_triangleIdToElementIdMap = createTriangleIdToElementIdMap(mesh);
	}

	return true;
}

bool Fem3DRepresentation::doInitialize()
{
	for (auto& element : m_fem->getElements())
	{
		std::shared_ptr<FemElement> femElement;
		femElement = FemElement::getFactory().create(getFemElementType(), element);
		m_femElements.push_back(femElement);
	}

	return FemRepresentation::doInitialize();
}

std::shared_ptr<Localization> Fem3DRepresentation::createNodeLocalization(size_t nodeId)
{
	DataStructures::IndexedLocalCoordinate coordinate;

	SURGSIM_ASSERT(nodeId >= 0 && nodeId < getCurrentState()->getNumNodes()) << "Invalid node id";

	// Look for any element that contains this node
	bool foundNodeId = false;
	for (size_t elementId = 0; elementId < getNumFemElements(); elementId++)
	{
		auto element = getFemElement(elementId);
		auto found = std::find(element->getNodeIds().begin(), element->getNodeIds().end(), nodeId);
		if (found != element->getNodeIds().end())
		{
			coordinate.index = elementId;
			coordinate.coordinate.setZero(element->getNumNodes());
			coordinate.coordinate[found - element->getNodeIds().begin()] = 1.0;
			foundNodeId = true;
			break;
		}
	}
	SURGSIM_ASSERT(foundNodeId) << "Could not find any element containing the node " << nodeId;

	// Fem3DLocalization will verify the coordinate (2nd parameter) based on
	// the Fem3DRepresentation passed as 1st parameter.
	return std::make_shared<Fem3DLocalization>(
				std::static_pointer_cast<Physics::Representation>(getSharedPtr()), coordinate);
}

std::shared_ptr<Localization> Fem3DRepresentation::createTriangleLocalization(
		const DataStructures::IndexedLocalCoordinate& location)
{
	DataStructures::IndexedLocalCoordinate coordinate;
	size_t triangleId = location.index;
	const Math::Vector& triangleCoord = location.coordinate;

	auto deformableCollision =
			std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);
	SURGSIM_ASSERT(deformableCollision != nullptr)
			<< "Triangle localization cannot be created if the DeformableCollisionRepresentation is not correctly set.";

	// Find the vertex ids of the triangle.
	auto mesh = std::dynamic_pointer_cast<Math::MeshShape>(deformableCollision->getShape());
	auto triangleVertices = mesh->getTriangle(triangleId).verticesId;

	// Find the vertex ids of the corresponding FemNode.
	// Get FemElement id from the triangle id.
	SURGSIM_ASSERT(m_triangleIdToElementIdMap.count(triangleId) == 1) <<
																		 "Triangle must be mapped to an fem element.";

	size_t elementId = m_triangleIdToElementIdMap[triangleId];
	std::shared_ptr<FemElement> element = getFemElement(elementId);

	auto elementVertices = element->getNodeIds();

	// Find the mapping between triangleVertices and elementVertices.
	std::vector<size_t> indices;
	indices.reserve(elementVertices.size());
	for (size_t i = 0; i < elementVertices.size(); ++i)
	{
		indices.push_back(3);
		for (int j = 0; j < 3; ++j)
		{
			if (triangleVertices[j] == elementVertices[i])
			{
				indices[i] = j;
				break;
			}
		}
	}

	// Create the natural coordinate.
	Math::Vector4d barycentricCoordinate(triangleCoord[0], triangleCoord[1], triangleCoord[2], 0.0);
	coordinate.index = elementId;
	coordinate.coordinate.resize(elementVertices.size());
	for (size_t i = 0; i < elementVertices.size(); ++i)
	{
		coordinate.coordinate[i] = barycentricCoordinate[indices[i]];
	}

	// Fem3DLocalization will verify the coordinate (2nd parameter) based on
	// the Fem3DRepresentation passed as 1st parameter.
	return std::make_shared<Fem3DLocalization>(
				std::static_pointer_cast<Physics::Representation>(getSharedPtr()), coordinate);
}

std::shared_ptr<Localization> Fem3DRepresentation::createElementLocalization(
		const DataStructures::IndexedLocalCoordinate& location)
{
	return std::make_shared<Fem3DLocalization>(
				std::static_pointer_cast<Physics::Representation>(getSharedPtr()), location);
}

std::shared_ptr<Localization> Fem3DRepresentation::createLocalization(const DataStructures::Location& location)
{
	if (location.index.hasValue())
	{
		return createNodeLocalization(*location.index);
	}
	else if (location.triangleMeshLocalCoordinate.hasValue())
	{
		return createTriangleLocalization(*location.triangleMeshLocalCoordinate);
	}
	else if (location.elementMeshLocalCoordinate.hasValue())
	{
		return createElementLocalization(*location.elementMeshLocalCoordinate);
	}

	SURGSIM_FAILURE() << "Localization cannot be created without a mesh-based location (node, triangle or element).";

	return nullptr;
}

void Fem3DRepresentation::transformState(std::shared_ptr<Math::OdeState> state,
										 const Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

} // namespace Physics
} // namespace SurgSim
