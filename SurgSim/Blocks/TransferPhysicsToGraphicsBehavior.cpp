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

#include "SurgSim/Blocks/TransferPhysicsToGraphicsBehavior.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace
{
template<class Data>
void updateVertex(const std::shared_ptr<SurgSim::Math::OdeState>& state,
				  std::shared_ptr<SurgSim::DataStructures::Vertices<Data>> target)
{
	if (target->getNumVertices() == 0)
	{
		for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
		{
			typename SurgSim::DataStructures::Vertices<Data>::VertexType vertex(state->getPosition(nodeId));
			target->addVertex(vertex);
		}
		SURGSIM_ASSERT(target->getNumVertices() == state->getNumNodes());
	}
}
}

namespace SurgSim
{

namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::TransferPhysicsToGraphicsBehavior,
				 TransferPhysicsToGraphicsBehavior);

TransferPhysicsToGraphicsBehavior::TransferPhysicsToGraphicsBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToGraphicsBehavior,
		std::shared_ptr<SurgSim::Framework::Component>, Source, getSource, setSource);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToGraphicsBehavior,
		std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget, setTarget);
}

void TransferPhysicsToGraphicsBehavior::setSource(
	const std::shared_ptr<SurgSim::Framework::Component>& source)
{
	SURGSIM_ASSERT(nullptr != source) << __FUNCTION__ << " 'source' can not be nullptr.";
	auto deformable = std::dynamic_pointer_cast<SurgSim::Physics::DeformableRepresentation>(source);
	SURGSIM_ASSERT(nullptr != deformable) << __FUNCTION__ << " 'source' is not a " <<
		"SurgSim::Physics::DeformableRepresentation.";

	m_source = deformable;
}

void TransferPhysicsToGraphicsBehavior::setTarget(
	const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << __FUNCTION__ << " 'target' can not be nullptr.";
	auto mesh = std::dynamic_pointer_cast<SurgSim::Graphics::MeshRepresentation>(target);
	auto pointCloud = std::dynamic_pointer_cast<SurgSim::Graphics::PointCloudRepresentation>(target);
	SURGSIM_ASSERT(nullptr != mesh || nullptr != pointCloud) << __FUNCTION__ <<
			" 'target' is not a SurgSim::Graphics::MeshRepresentation or SurgSim::Graphics::PointCloudRepresentation.";

	m_target = std::static_pointer_cast<SurgSim::Graphics::Representation>(target);
}

std::shared_ptr<SurgSim::Framework::Component> TransferPhysicsToGraphicsBehavior::getSource() const
{
	return m_source;
}

std::shared_ptr<SurgSim::Framework::Component> TransferPhysicsToGraphicsBehavior::getTarget() const
{
	return m_target;
}

void TransferPhysicsToGraphicsBehavior::update(double dt)
{
	auto state = m_source->getFinalState();
	auto numNodes = state->getNumNodes();

	auto mesh = std::dynamic_pointer_cast<SurgSim::Graphics::MeshRepresentation>(m_target);
	auto pointCloud = std::dynamic_pointer_cast<SurgSim::Graphics::PointCloudRepresentation>(m_target);

	std::vector<SurgSim::Math::Vector3d> positions(numNodes);
	for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
	{
		positions[nodeId] = state->getPosition(nodeId);
	}

	if (nullptr != mesh)
	{
		SURGSIM_ASSERT(nullptr == pointCloud);
		auto target = mesh->getMesh();
		target->setVertexPositions(positions);
	}
	else
	{
		SURGSIM_ASSERT(nullptr != pointCloud);
		auto target = pointCloud->getVertices();
		target->setVertexPositions(positions);
	}
}

bool TransferPhysicsToGraphicsBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToGraphicsBehavior::doWakeUp()
{
	auto state = m_source->getFinalState();
	auto numNodes = state->getNumNodes();

	auto mesh = std::dynamic_pointer_cast<SurgSim::Graphics::MeshRepresentation>(m_target);
	auto pointCloud = std::dynamic_pointer_cast<SurgSim::Graphics::PointCloudRepresentation>(m_target);
	if (nullptr != mesh)
	{
		auto target = std::dynamic_pointer_cast
						<SurgSim::DataStructures::Vertices<SurgSim::Graphics::VertexData>>(mesh->getMesh());
		SURGSIM_ASSERT(nullptr == pointCloud && nullptr != target);

		updateVertex(state, target);
	}
	else
	{
		SURGSIM_ASSERT(nullptr != pointCloud);
		auto target = pointCloud->getVertices();

		updateVertex(state, target);
	}
	return true;
}

}; //namespace Blocks
}; //namespace SurgSim