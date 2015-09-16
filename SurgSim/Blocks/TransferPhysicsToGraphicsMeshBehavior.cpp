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

#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Aabb.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

using SurgSim::Framework::checkAndConvert;

namespace SurgSim
{

namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior,
				 TransferPhysicsToGraphicsMeshBehavior);

TransferPhysicsToGraphicsMeshBehavior::TransferPhysicsToGraphicsMeshBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToGraphicsMeshBehavior,
									  std::shared_ptr<SurgSim::Framework::Component>, Source, getSource, setSource);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToGraphicsMeshBehavior,
									  std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget, setTarget);

	// Enable full serialization on the index map type, but need to deal with overloaded functions
	{
		typedef std::vector<std::pair<size_t, size_t>> ParamType;
		typedef TransferPhysicsToGraphicsMeshBehavior ClassType;

		// Deal with the overloaded function, by casting to explicit function type
		auto getter = (ParamType(ClassType::*)(void) const)&ClassType::getIndexMap;
		auto setter = (void(ClassType::*)(const ParamType&))&ClassType::setIndexMap;

		setAccessors("IndexMap", std::bind(getter, this),
					 std::bind(setter, this, std::bind(SurgSim::Framework::convert<ParamType>, std::placeholders::_1)));

		setSerializable("IndexMap",
						std::bind(&YAML::convert<ParamType>::encode, std::bind(getter, this)),
						std::bind(setter, this, std::bind(&YAML::Node::as<ParamType>, std::placeholders::_1)));
	}

	// Enable a setter to take the names of two mesh files to create the index map
	{
		typedef std::pair<std::string, std::string> ParamType;
		typedef TransferPhysicsToGraphicsMeshBehavior ClassType;
		auto setter = (void(ClassType::*)(const ParamType&))&ClassType::setIndexMap;

		setSetter("IndexMapMeshNames",
				  std::bind(setter, this, std::bind(SurgSim::Framework::convert<ParamType>, std::placeholders::_1)));

		setDecoder("IndexMapMeshNames",
				   std::bind(setter, this, std::bind(&YAML::Node::as<ParamType>, std::placeholders::_1)));
	}

	// Enable a setter to take two meshes to create the index map
	{
		typedef std::pair<std::shared_ptr<Framework::Asset>, std::shared_ptr<Framework::Asset>> ParamType;
		typedef TransferPhysicsToGraphicsMeshBehavior ClassType;
		auto setter = (void(ClassType::*)(const ParamType&))&ClassType::setIndexMap;

		setSetter("IndexMapMeshes",
				  std::bind(setter, this, std::bind(SurgSim::Framework::convert<ParamType>, std::placeholders::_1)));

		setDecoder("IndexMapMeshes",
				   std::bind(setter, this, std::bind(&YAML::Node::as<ParamType>, std::placeholders::_1)));
	}


}

void TransferPhysicsToGraphicsMeshBehavior::setSource(const std::shared_ptr<SurgSim::Framework::Component>& source)
{
	SURGSIM_ASSERT(nullptr != source) << " 'source' can not be nullptr.";
	m_source = checkAndConvert<SurgSim::Physics::DeformableRepresentation>(
				   source, "SurgSim::Physics::DeformableRepresentation");
}

void TransferPhysicsToGraphicsMeshBehavior::setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << " 'target' can not be nullptr.";
	m_target = checkAndConvert<SurgSim::Graphics::MeshRepresentation>(
				   target, "SurgSim::Graphics::MeshRepresentation");
}

std::shared_ptr<SurgSim::Physics::DeformableRepresentation> TransferPhysicsToGraphicsMeshBehavior::getSource() const
{
	return m_source;
}

std::shared_ptr<SurgSim::Graphics::MeshRepresentation> TransferPhysicsToGraphicsMeshBehavior::getTarget() const
{
	return m_target;
}

void TransferPhysicsToGraphicsMeshBehavior::update(double dt)
{
	auto state = m_source->getFinalState();

	if (m_indexMap.empty())
	{
		auto mesh = m_target->getMesh();
		for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
		{
			mesh->setVertexPosition(nodeId, state->getPosition(nodeId));
		}
	}
	else
	{
		auto mesh = m_target->getMesh();
		for (const auto& mapping : m_indexMap)
		{
			mesh->setVertexPosition(mapping.second, state->getPosition(mapping.first));
		}
	}

	m_target->getMesh()->dirty();
}

bool TransferPhysicsToGraphicsMeshBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToGraphicsMeshBehavior::doWakeUp()
{
	auto state = m_source->getFinalState();
	auto target = m_target->getMesh();

	if (target->getNumVertices() == 0)
	{
		for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
		{
			SurgSim::Graphics::Mesh::VertexType vertex(state->getPosition(nodeId));
			target->addVertex(vertex);
		}
	}

	return true;
}

void TransferPhysicsToGraphicsMeshBehavior::setIndexMap(
	const std::shared_ptr<DataStructures::TriangleMeshPlain>& source,
	const std::shared_ptr<DataStructures::TriangleMeshPlain>& target)
{
	setIndexMap(generateIndexMap(source, target));
}

void TransferPhysicsToGraphicsMeshBehavior::setIndexMap(const std::string& sourceName , const std::string& targetName)
{
	auto source = std::make_shared<DataStructures::TriangleMeshPlain>();
	source->load(sourceName);


	auto target = std::make_shared<DataStructures::TriangleMeshPlain>();
	target->load(targetName);

	setIndexMap(source, target);
}

void TransferPhysicsToGraphicsMeshBehavior::setIndexMap(const std::pair<std::string, std::string>& fileName)
{
	setIndexMap(fileName.first, fileName.second);
}

void TransferPhysicsToGraphicsMeshBehavior::setIndexMap(
	const std::pair<std::shared_ptr<Framework::Asset>, std::shared_ptr<Framework::Asset>>& meshes)
{
	auto first = std::dynamic_pointer_cast<DataStructures::TriangleMeshPlain>(meshes.first);
	auto second = std::dynamic_pointer_cast<DataStructures::TriangleMeshPlain>(meshes.second);

	SURGSIM_ASSERT(first != nullptr)  << "Incoming first asset not TriangleMeshPlain";
	SURGSIM_ASSERT(second != nullptr)  << "Incoming second asset not TriangleMeshPlain";

	setIndexMap(first, second);
}

std::vector<std::pair<size_t, size_t>> generateIndexMap(
										const std::shared_ptr<DataStructures::TriangleMeshPlain>& source,
										const std::shared_ptr<DataStructures::TriangleMeshPlain>& target)
{
	SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getDefaultLogger())
			<< "Building map";

	SURGSIM_ASSERT(source->getNumVertices() > 0 && target->getNumVertices() > 0)
			<< "Can't build correspondence map, meshes are missing vertices";


	// Caclulate AABB for Mesh
	const auto& vertices = target->getVertices();
	SurgSim::Math::Aabbd bounds;

	for (const auto& vertex : vertices)
	{
		// m_updateMesh.addVertex(SurgSim::Graphics::Mesh::VertexType(m_, SurgSim::Graphics::VertexData()));
		bounds.extend(vertex.position);
	}

	bounds.extend(bounds.max() * 1.1);
	bounds.extend(bounds.min() * 1.1);

	// Add All vertices to grid
	// 100 Seems like a reasonable value ...
	SurgSim::Math::Vector3d cellSize(bounds.diagonal() / 100.0);
	SurgSim::DataStructures::Grid<size_t, 3> grid(cellSize, bounds);

	size_t index = 0;
	for (const auto& vertex : vertices)
	{
		grid.addElement(index++, vertex.position);
	}

	std::vector<std::pair<size_t, size_t>> indexMap;

	// For each state node query grid for neighbors
	indexMap.reserve(source->getNumVertices());
	for (size_t nodeId = 0; nodeId < source->getNumVertices(); ++nodeId)
	{
		bool hasNeighbors = false;
		auto position = source->getVertexPosition(nodeId);

		// make a copy
		const std::vector<size_t>& neighbors = grid.getNeighbors(position);
		// For each neighbor check if same, if yes add to map

		for (auto neighborIndex : neighbors)
		{
			if (position.isApprox(vertices[neighborIndex].position))
			{
				indexMap.push_back(std::make_pair(nodeId, neighborIndex));
				hasNeighbors = true;
			}
		}

		if (! hasNeighbors)
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Vascular"))
					<< "No coincident point found for node with index " << index;

		}
	}

	return indexMap;
}

}; //namespace Blocks
}; //namespace SurgSim