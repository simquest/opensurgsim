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
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Collision/Location.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
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
	FemRepresentation(name), m_doLoadFile(false)
{
	// Reminder: m_numDofPerNode is held by DeformableRepresentation
	// but needs to be set by all concrete derived classes
	m_numDofPerNode = 3;

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Fem3DRepresentation, std::string, Filename, getFilename, setFilename);
}

Fem3DRepresentation::~Fem3DRepresentation()
{
}

RepresentationType Fem3DRepresentation::getType() const
{
	return REPRESENTATION_TYPE_FEM3D;
}

void Fem3DRepresentation::setFilename(const std::string& filename)
{
	m_filename = filename;

	m_doLoadFile = !m_filename.empty();
}

const std::string& Fem3DRepresentation::getFilename() const
{
	return m_filename;
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
							   { return std::includes(femElementSorted.begin(), femElementSorted.end(),
													  triangleSorted.begin(), triangleSorted.end()); };

	auto& meshTriangles = mesh.getTriangles();
	for (auto triangle = meshTriangles.cbegin(); triangle != meshTriangles.cend(); ++triangle)
	{
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

std::shared_ptr<Localization> Fem3DRepresentation::createLocalization(const SurgSim::Collision::Location& location)
{
	SURGSIM_ASSERT(location.meshLocalCoordinate.hasValue())
		<< "Localization cannot be created if the triangle ID is not available.";

	SURGSIM_ASSERT(location.meshLocalCoordinate.getValue().naturalCoordinate.size() == 3)
		<< "Localization has incorrect size for the barycentric coordinates.";

	// Find the vertex ids of the triangle.
	size_t triangleId = location.meshLocalCoordinate.getValue().elementId;
	SurgSim::Math::Vector4d triangleBarycentricCoordinate4(location.meshLocalCoordinate.getValue().naturalCoordinate[0],
														   location.meshLocalCoordinate.getValue().naturalCoordinate[1],
														   location.meshLocalCoordinate.getValue().naturalCoordinate[2],
														   0.0);
	auto deformableCollisionRepresentation
		= std::dynamic_pointer_cast<DeformableCollisionRepresentation>(m_collisionRepresentation);

	SURGSIM_ASSERT(deformableCollisionRepresentation != nullptr)
		<< "Localization cannot be created if the DeformableCollisionRepresentation is not correctly set.";

	auto triangleVertexIds = deformableCollisionRepresentation->getMesh()->getTriangle(triangleId).verticesId;

	// Find the vertex ids of the corresponding FemNode.
	// Get FemElement id from the triangle id.
	SURGSIM_ASSERT(m_triangleIdToElementIdMap.count(triangleId) == 1) << "Triangle must be mapped to an fem element.";

	size_t elementId = m_triangleIdToElementIdMap[triangleId];
	std::shared_ptr<FemElement> element = getFemElement(elementId);

	auto elementVertexIds = element->getNodeIds();

	// Find the mapping between triangleVertexIds and elementVertexIds.
	std::array<int, 4> indices;
	for (int i = 0; i < 4; ++i)
	{
		indices[i] = 3;
		for (int j = 0; j < 3; ++j)
		{
			if (triangleVertexIds[j] == elementVertexIds[i])
			{
				indices[i] = j;
				break;
			}
		}
	}

	// Create the natual coordinate.
	FemRepresentationCoordinate coordinate;
	coordinate.elementId = elementId;
	coordinate.naturalCoordinate = SurgSim::Math::Vector4d(triangleBarycentricCoordinate4[indices[0]],
														   triangleBarycentricCoordinate4[indices[1]],
														   triangleBarycentricCoordinate4[indices[2]],
														   triangleBarycentricCoordinate4[indices[3]]);

	// Fem3DRepresentationLocalization::setLocalPosition verifies argument based on its Representation.
	auto result = std::make_shared<Fem3DRepresentationLocalization>();
	result->setRepresentation(std::static_pointer_cast<SurgSim::Physics::Representation>(getSharedPtr()));
	result->setLocalPosition(coordinate);

	return result;
}

bool Fem3DRepresentation::doInitialize()
{
	bool result = true;
	if (m_doLoadFile)
	{
		if (!loadFile())
		{
			SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << __FUNCTION__ << "Nothing loaded from file " << m_filename;
			result = false;
		}
	}

	return result && FemRepresentation::doInitialize();
}

bool Fem3DRepresentation::loadFile()
{
	bool result = true;
	std::string filePath = getRuntime()->getApplicationData()->findFile(m_filename);

	if (m_filename.empty() || filePath.empty())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) <<
			"Fem3DRepresentation::doInitialize(): file " << m_filename << " can not be found.";
		result = false;
	}
	else
	{
		SurgSim::DataStructures::PlyReader reader(filePath);
		auto thisAsSharedPtr = std::static_pointer_cast<Fem3DRepresentation>(getSharedPtr());
		auto readerDelegate = std::make_shared<Fem3DRepresentationPlyReaderDelegate>(thisAsSharedPtr);

		if (!reader.isValid())
		{
			SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Fem3DRepresentation::doInitialize(): " <<
				"File " << filePath << " is not valid.";
			result = false;
		}

		if (result && !reader.setDelegate(readerDelegate))
		{
			SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Fem3DRepresentation::doInitialize(): " <<
				"File " << filePath << " is not an acceptable PLY.";
			result = false;
		}

		if (result)
		{
			// PlyReader::parseFile loads the fem into the shared_ptr passed to the readerDelegate constructor.
			reader.parseFile();
			m_doLoadFile = false;
		}
	}

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
