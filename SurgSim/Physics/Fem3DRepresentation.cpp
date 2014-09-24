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
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/FemElement.h"

using SurgSim::Framework::Logger;

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

RepresentationType Fem3DRepresentation::getType() const
{
	return REPRESENTATION_TYPE_FEM3D;
}

void Fem3DRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		SurgSim::Math::Vector& generalizedForce,
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

	std::shared_ptr<Fem3DRepresentationLocalization> localization3D =
		std::dynamic_pointer_cast<Fem3DRepresentationLocalization>(localization);
	SURGSIM_ASSERT(localization3D != nullptr) << "Invalid localization type (not a Fem3DRepresentationLocalization)";

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
					m_externalGeneralizedStiffness.block(dofPerNode * nodeId1,
														 dofPerNode * nodeId2,
														 dofPerNode, dofPerNode)
					+= coordinate[index1] * coordinate[index2] * K;
				}
				if (D.size() != 0)
				{
					m_externalGeneralizedDamping.block(dofPerNode * nodeId1,
													   dofPerNode * nodeId2,
													   dofPerNode, dofPerNode)
					+= coordinate[index1] * coordinate[index2] * D;
				}
				index2++;
			}

			index1++;
		}
	}
}

std::shared_ptr<FemPlyReaderDelegate> Fem3DRepresentation::getDelegate()
{
	auto thisAsSharedPtr = std::static_pointer_cast<Fem3DRepresentation>(shared_from_this());
	auto readerDelegate = std::make_shared<Fem3DPlyReaderDelegate>(thisAsSharedPtr);

	return readerDelegate;
}

std::unordered_map<size_t, size_t> Fem3DRepresentation::createTriangleIdToElementIdMap(
	const SurgSim::DataStructures::TriangleMesh& mesh)
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

	auto& meshTriangles = mesh.getTriangles();
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

	auto deformableCollisionRepresentation
		= std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);

	if (deformableCollisionRepresentation != nullptr)
	{
		m_triangleIdToElementIdMap = createTriangleIdToElementIdMap(*deformableCollisionRepresentation->getMesh());
	}

	return true;
}

std::shared_ptr<Localization> Fem3DRepresentation::createLocalization(const SurgSim::DataStructures::Location& location)
{
	SURGSIM_ASSERT(location.meshLocalCoordinate.hasValue())
			<< "Localization cannot be created if the triangle ID is not available.";

	SURGSIM_ASSERT(location.meshLocalCoordinate.getValue().coordinate.size() == 3)
			<< "Localization has incorrect size for the barycentric coordinates.";

	auto deformableCollisionRepresentation
		= std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);

	SURGSIM_ASSERT(deformableCollisionRepresentation != nullptr)
			<< "Localization cannot be created if the DeformableCollisionRepresentation is not correctly set.";

	// Find the vertex ids of the triangle.
	size_t triangleId = location.meshLocalCoordinate.getValue().index;
	auto triangleVertices = deformableCollisionRepresentation->getMesh()->getTriangle(triangleId).verticesId;

	// Find the vertex ids of the corresponding FemNode.
	// Get FemElement id from the triangle id.
	SURGSIM_ASSERT(m_triangleIdToElementIdMap.count(triangleId) == 1) << "Triangle must be mapped to an fem element.";

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
	SurgSim::Math::Vector4d barycentricCoordinate(location.meshLocalCoordinate.getValue().coordinate[0],
			location.meshLocalCoordinate.getValue().coordinate[1],
			location.meshLocalCoordinate.getValue().coordinate[2],
			0.0);
	SurgSim::DataStructures::IndexedLocalCoordinate coordinate;
	coordinate.index = elementId;
	coordinate.coordinate.resize(elementVertices.size());
	for (size_t i = 0; i < elementVertices.size(); ++i)
	{
		coordinate.coordinate[i] = barycentricCoordinate[indices[i]];
	}

	// Fem3DRepresentationLocalization will verify the coordinate (2nd parameter) based on
	// the Fem3DRepresentation passed as 1st parameter.
	auto result = std::make_shared<Fem3DRepresentationLocalization>(
					  std::static_pointer_cast<SurgSim::Physics::Representation>(getSharedPtr()), coordinate);

	return result;
}

void Fem3DRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
		const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

} // namespace Physics
} // namespace SurgSim
