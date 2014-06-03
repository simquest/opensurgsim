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
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/MeshRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/Representation.h"
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
	auto deformable = std::dynamic_pointer_cast<SurgSim::Physics::DeformableRepresentation>(source);
	SURGSIM_ASSERT(nullptr != deformable) << "TransferVerticesFromPhysicsToGraphicsBehavior can only take a " <<
		"SurgSim::Physics::DeformableRepresentation as source.";
	m_source = deformable;
}

void TransferPhysicsToGraphicsMeshBehavior::setTarget(
	const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	auto mesh = std::dynamic_pointer_cast<SurgSim::Graphics::MeshRepresentation>(target);
	SURGSIM_ASSERT(nullptr != target) << "TransferVerticesFromPhysicsToGraphicsBehavior: 'target' can not be nullptr.";
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
	// Note that transfer is called without initialization on
	// This should have been done in doWakeUp already
	transfer();
}

bool TransferPhysicsToGraphicsMeshBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToGraphicsMeshBehavior::doWakeUp()
{
	// Note that transfer is called with initialization on
	// This is done in doWakeUp as an external data structure will be initialized (Vertices)
	transfer(true);
	return true;
}

void TransferPhysicsToGraphicsMeshBehavior::transfer(bool doInitialization)
{
	auto finalState = m_source->getFinalState();
	const unsigned int numNodes = finalState->getNumNodes();

	auto target = m_target->getMesh();

	// If initialization is requested and vertices is empty, let's populate it properly
	if (doInitialization == true && target->getNumVertices() == 0 && numNodes != 0)
	{
		for (unsigned int nodeId = 0; nodeId < numNodes; nodeId++)
		{
			SurgSim::DataStructures::Vertex<SurgSim::Graphics::VertexData> v(finalState->getPosition(nodeId));
			target->addVertex(v);
		}
	}
	else if (target->getNumVertices() == numNodes)
	{
		for (unsigned int nodeId = 0; nodeId < numNodes; nodeId++)
		{
			target->setVertexPosition(nodeId, finalState->getPosition(nodeId));
		}
	}
}

}; //namespace Blocks
}; //namespace SurgSim
