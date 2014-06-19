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
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior);
}

namespace SurgSim
{

namespace Blocks
{

TransferPhysicsToGraphicsMeshBehavior::TransferPhysicsToGraphicsMeshBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToGraphicsMeshBehavior,
		std::shared_ptr<SurgSim::Framework::Component>, Source, getSource, setSource);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToGraphicsMeshBehavior,
		std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget, setTarget);
}

void TransferPhysicsToGraphicsMeshBehavior::setSource(
	const std::shared_ptr<SurgSim::Framework::Component>& source)
{
	SURGSIM_ASSERT(nullptr != source) << __FUNCTION__ << " 'source' can not be nullptr.";
	auto deformable = std::dynamic_pointer_cast<SurgSim::Physics::DeformableRepresentation>(source);
	SURGSIM_ASSERT(nullptr != deformable) << __FUNCTION__ << " 'source' is not a " <<
		"SurgSim::Physics::DeformableRepresentation.";

	m_source = deformable;
}

void TransferPhysicsToGraphicsMeshBehavior::setTarget(
	const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << __FUNCTION__ << " 'target' can not be nullptr.";
	auto mesh = std::dynamic_pointer_cast<SurgSim::Graphics::MeshRepresentation>(target);
	SURGSIM_ASSERT(nullptr != mesh) << __FUNCTION__ << " 'target' is not a SurgSim::Graphics::MeshRepresentation.";

	m_target = mesh;
}

std::shared_ptr<SurgSim::Framework::Component> TransferPhysicsToGraphicsMeshBehavior::getSource() const
{
	return m_source;
}
std::shared_ptr<SurgSim::Framework::Component> TransferPhysicsToGraphicsMeshBehavior::getTarget() const
{
	return m_target;
}

void TransferPhysicsToGraphicsMeshBehavior::update(double dt)
{
	auto finalState = m_source->getFinalState();
	auto numNodes = finalState->getNumNodes();
	auto target = m_target->getMesh();

	if (target->getNumVertices() == numNodes)
	{
		for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
		{
			target->setVertexPosition(nodeId, finalState->getPosition(nodeId));
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__ <<
			"Number of vertices contained by " << m_source->getName() << " and " <<
			m_target->getName() << " doesn't match. No vertex will be copied.";
	}
}

bool TransferPhysicsToGraphicsMeshBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToGraphicsMeshBehavior::doWakeUp()
{
	auto finalState = m_source->getFinalState();
	auto numNodes = finalState->getNumNodes();
	auto target = m_target->getMesh();

	if (target->getNumVertices() == 0)
	{
		for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
		{
			SurgSim::DataStructures::Vertex<SurgSim::Graphics::VertexData> vertex(finalState->getPosition(nodeId));
			target->addVertex(vertex);
		}
	}

	return true;
}

}; //namespace Blocks
}; //namespace SurgSim