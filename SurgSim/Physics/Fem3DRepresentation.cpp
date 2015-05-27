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

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

using SurgSim::Framework::Logger;
using SurgSim::Math::SparseMatrix;

namespace
{
void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform,
							   SurgSim::Math::Vector* x, bool rotationOnly = false)
{
	typedef SurgSim::Math::Vector::Index IndexType;

	IndexType numNodes = x->size() / 3;
	SURGSIM_ASSERT(numNodes * 3 == x->size()) <<
											"Unexpected number of dof in a Fem3D state vector (not a multiple of 3)";

	for (IndexType nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::Vector3d xi = SurgSim::Math::getSubVector(*x, nodeId, 3);
		SurgSim::Math::Vector3d xiTransformed;
		if (rotationOnly)
		{
			xiTransformed = transform.linear() * xi;
		}
		else
		{
			xiTransformed = transform * xi;
		}
		SurgSim::Math::setSubVector(xiTransformed, nodeId, 3, x);
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
	// Reminder: m_numDofPerNode is held by DeformableRepresentation
	// but needs to be set by all concrete derived classes
	m_numDofPerNode = 3;
}

Fem3DRepresentation::~Fem3DRepresentation()
{
}

void Fem3DRepresentation::loadFem(const std::string& fileName)
{
	m_filename = fileName;
	auto mesh = std::make_shared<Fem3D>();
	mesh->load(fileName);
	setMesh(mesh);
	m_isFemLoaded = true;
}

void Fem3DRepresentation::setMesh(std::shared_ptr<Framework::Asset> mesh)
{
	auto femMesh = std::dynamic_pointer_cast<Fem3D>(mesh);
	SURGSIM_ASSERT(femMesh != nullptr)
			<< "Mesh for Fem3DRepresentation needs to be a SurgSim::Physics::Fem3D";
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
	FemRepresentation::setInitialState(state);
}

std::shared_ptr<Fem3D> Fem3DRepresentation::getMesh() const
{
	return m_fem;
}

void Fem3DRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
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

	std::shared_ptr<Fem3DLocalization> localization3D =
			std::dynamic_pointer_cast<Fem3DLocalization>(localization);
	SURGSIM_ASSERT(localization3D != nullptr) << "Invalid localization type (not a Fem3DLocalization)";

	const size_t elementId = localization3D->getLocalPosition().index;
	const SurgSim::Math::Vector& coordinate = localization3D->getLocalPosition().coordinate;
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

std::unordered_map<size_t, size_t> Fem3DRepresentation::createTriangleIdToElementIdMap(
		std::shared_ptr<const SurgSim::Math::MeshShape> mesh)
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
		auto mesh = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(deformableCollision->getShape());
		m_triangleIdToElementIdMap = createTriangleIdToElementIdMap(mesh);
	}

	return true;
}

bool Fem3DRepresentation::doInitialize()
{
	if (!m_filename.empty() && !isFemLoaded())
	{
		loadFem(m_filename);
	}

	// If mesh is set, create the FemElements
	if (m_fem != nullptr)
	{
		for (auto& element : m_fem->getFemElements())
		{
			std::shared_ptr<FemElement> femElement;
			if (m_femElementOverrideType.empty())
			{
				femElement = FemElement::getFactory().create(element->type, element);
			}
			else
			{
				femElement = FemElement::getFactory().create(m_femElementOverrideType, element);
			}

			m_femElements.push_back(femElement);
		}
	}

	return FemRepresentation::doInitialize();
}

std::shared_ptr<Localization> Fem3DRepresentation::createNodeLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location)
{
	SurgSim::DataStructures::IndexedLocalCoordinate coordinate;
	size_t nodeId = location.index;

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
				std::static_pointer_cast<SurgSim::Physics::Representation>(getSharedPtr()), coordinate);
}

std::shared_ptr<Localization> Fem3DRepresentation::createTriangleLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location)
{
	SurgSim::DataStructures::IndexedLocalCoordinate coordinate;
	size_t triangleId = location.index;
	const SurgSim::Math::Vector& triangleCoord = location.coordinate;

	auto deformableCollision =
			std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);
	SURGSIM_ASSERT(deformableCollision != nullptr)
			<< "Triangle localization cannot be created if the DeformableCollisionRepresentation is not correctly set.";

	// Find the vertex ids of the triangle.
	auto mesh = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(deformableCollision->getShape());
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
	SurgSim::Math::Vector4d barycentricCoordinate(triangleCoord[0], triangleCoord[1], triangleCoord[2], 0.0);
	coordinate.index = elementId;
	coordinate.coordinate.resize(elementVertices.size());
	for (size_t i = 0; i < elementVertices.size(); ++i)
	{
		coordinate.coordinate[i] = barycentricCoordinate[indices[i]];
	}

	// Fem3DLocalization will verify the coordinate (2nd parameter) based on
	// the Fem3DRepresentation passed as 1st parameter.
	return std::make_shared<Fem3DLocalization>(
				std::static_pointer_cast<SurgSim::Physics::Representation>(getSharedPtr()), coordinate);
}

std::shared_ptr<Localization> Fem3DRepresentation::createElementLocalization(
		const SurgSim::DataStructures::IndexedLocalCoordinate& location)
{
	return std::make_shared<Fem3DLocalization>(
				std::static_pointer_cast<SurgSim::Physics::Representation>(getSharedPtr()), location);
}

std::shared_ptr<Localization> Fem3DRepresentation::createLocalization(const SurgSim::DataStructures::Location& location)
{
	if (location.nodeMeshLocalCoordinate.hasValue())
	{
		return createNodeLocalization(location.nodeMeshLocalCoordinate.getValue());
	}
	else if (location.triangleMeshLocalCoordinate.hasValue())
	{
		return createTriangleLocalization(location.triangleMeshLocalCoordinate.getValue());
	}
	else if (location.elementMeshLocalCoordinate.hasValue())
	{
		return createElementLocalization(location.elementMeshLocalCoordinate.getValue());
	}

	SURGSIM_FAILURE() << "Localization cannot be created without a mesh-based location (node, triangle or element).";

	return nullptr;
}

void Fem3DRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
										 const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

} // namespace Physics
} // namespace SurgSim
