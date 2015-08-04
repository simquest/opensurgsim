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

#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace SurgSim
{

namespace Blocks
{


TransferPhysicsToVerticesBehavior::TransferPhysicsToVerticesBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{

}

void TransferPhysicsToVerticesBehavior::setSource(const std::shared_ptr<SurgSim::Framework::Component>& source)
{
	SURGSIM_ASSERT(nullptr != source) << "'source' can not be nullptr.";
	m_source = Framework::checkAndConvert<SurgSim::Physics::DeformableRepresentation>(
				   source, "SurgSim::Physics::DeformableRepresentation");
}

void TransferPhysicsToVerticesBehavior::setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << "'target' can not be nullptr.";
	SURGSIM_ASSERT(target->isWriteable("Vertices")) << "'target'" << target->getFullName()
			<< "needs to accept 'Vertices'";
	m_target = target;
}

std::shared_ptr<SurgSim::Physics::DeformableRepresentation> TransferPhysicsToVerticesBehavior::getSource() const
{
	return m_source;
}

std::shared_ptr<SurgSim::Framework::Component> TransferPhysicsToVerticesBehavior::getTarget() const
{
	return m_target;
}

void TransferPhysicsToVerticesBehavior::update(double dt)
{
	auto state = m_source->getFinalState();
	for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
	{
		m_vertices.setVertexPosition(nodeId, state->getPosition(nodeId));
	}
	m_target->setValue("Vertices", m_vertices);
}

bool TransferPhysicsToVerticesBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToVerticesBehavior::doWakeUp()
{
	auto state = m_source->getFinalState();

	if (m_vertices.getNumVertices() == 0)
	{
		for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
		{
			DataStructures::Vertices<DataStructures::EmptyData>::VertexType vertex(state->getPosition(nodeId));
			m_vertices.addVertex(std::move(vertex));
		}
	}
	return true;
}

}
}
