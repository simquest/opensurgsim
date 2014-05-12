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

Fem3DRepresentation::Fem3DRepresentation(const std::string& name) :
	FemRepresentation(name), m_doLoadFile(false)
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

void Fem3DRepresentation::applyCorrection(double dt,
										  const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity)
{
	if (!isActive())
	{
		return;
	}

	m_currentState->getPositions() += deltaVelocity * dt;
	m_currentState->getVelocities() += deltaVelocity;

	if (!m_currentState->isValid())
	{
		deactivateAndReset();
	}
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

bool Fem3DRepresentation::loadFile()
{
	if (m_filename.empty())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Filename not set.";
		return false;
	}

	if (!m_doLoadFile)
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "File already loaded.";
		return false;
	}

	SurgSim::DataStructures::PlyReader reader(m_filename);
	auto thisAsSharedPtr = std::static_pointer_cast<Fem3DRepresentation>(getSharedPtr());
	auto readerDelegate = std::make_shared<Fem3DRepresentationPlyReaderDelegate>(thisAsSharedPtr);

	if (!reader.isValid())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "File " << m_filename << " is not valid.";
		return false;
	}

	if (!reader.setDelegate(readerDelegate))
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "File " << m_filename << " is not acceptable an PLY.";
		return false;
	}

	// PlyReader::parseFile loads the fem into the shared_ptr passed to the readerDelegate constructor.
	reader.parseFile();

	m_doLoadFile = false;
	return true;
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
	for (unsigned int i = 0; i < getNumFemElements(); ++i)
	{
		auto elementNodeIds = getFemElement(i)->getNodeIds();
		std::sort(elementNodeIds.begin(), elementNodeIds.end());
		// TODO(gsathyaseelan) 12th May 2014
		// Currently, converting the unsigned int from elementNodeIds to size_t explicitly.
		// Remove these when Physics API uses size_t instead of unsigned int.
		std::vector<size_t> elementNodeIdsConverted;
		for (auto it = elementNodeIds.cbegin(); it != elementNodeIds.cend(); ++it)
		{
			elementNodeIdsConverted.push_back(static_cast<size_t>(*it));
		}
		femElements.push_back(elementNodeIdsConverted);
	}

	std::array<unsigned int, 3> triangleSorted;
	auto doesIncludeTriangle = [&triangleSorted](std::vector<size_t>& femElementSorted)
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
	SURGSIM_ASSERT(location.triangleId.hasValue())
		<< "Localization cannot be created if the triangle ID is not available.";

	size_t triangleId = location.triangleId.getValue();
	SurgSim::Math::Vector globalPosition = location.globalPosition.getValue();

	// Get FemElement id from the triangle id.
	SURGSIM_ASSERT(m_triangleIdToElementIdMap.count(triangleId) == 1) << "Triangle must be mapped to an fem element.";

	unsigned int elementId = static_cast<unsigned int>(m_triangleIdToElementIdMap[triangleId]);
	std::shared_ptr<FemElement> element = getFemElement(elementId);

	FemRepresentationCoordinate coordinate;
	coordinate.elementId = elementId;
	coordinate.naturalCoordinate = element->computeNaturalCoordinate(*m_currentState, globalPosition);

	// Fem3DRepresentationLocalization::setLocalPosition verifies argument based on its Representation.
	auto result = std::make_shared<Fem3DRepresentationLocalization>();
	result->setRepresentation(std::static_pointer_cast<SurgSim::Physics::Representation>(getSharedPtr()));
	result->setLocalPosition(coordinate);

	return result;
}

bool Fem3DRepresentation::doInitialize()
{
	if (m_doLoadFile && !loadFile())
	{
		SURGSIM_LOG_SEVERE(Logger::getDefaultLogger()) << "Failed to initialize from file " << m_filename;
		return false;
	}

	return FemRepresentation::doInitialize();
}

void Fem3DRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
	const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

void Fem3DRepresentation::deactivateAndReset(void)
{
	SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), DEBUG)
		<< getName() << " deactivated and reset:" << std::endl
		<< "position=(" << m_currentState->getPositions() << ")" << std::endl
		<< "velocity=(" << m_currentState->getVelocities() << ")" << std::endl;

	resetState();
	setIsActive(false);
}

} // namespace Physics

} // namespace SurgSim
