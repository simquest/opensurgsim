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

#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

using SurgSim::Framework::checkAndConvert;

namespace SurgSim
{

namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::TransferPhysicsToPointCloudBehavior,
				 TransferPhysicsToPointCloudBehavior);

TransferPhysicsToPointCloudBehavior::TransferPhysicsToPointCloudBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToPointCloudBehavior,
									  std::shared_ptr<SurgSim::Framework::Component>, Source, getSource, setSource);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferPhysicsToPointCloudBehavior,
									  std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget, setTarget);
}

void TransferPhysicsToPointCloudBehavior::setSource(const std::shared_ptr<SurgSim::Framework::Component>& source)
{
	SURGSIM_ASSERT(nullptr != source) << "'source' can not be nullptr.";
	m_source = checkAndConvert<SurgSim::Physics::DeformableRepresentation>(
				   source, "SurgSim::Physics::DeformableRepresentation");
}

void TransferPhysicsToPointCloudBehavior::setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << "'target' can not be nullptr.";
	m_target = checkAndConvert<SurgSim::Graphics::PointCloudRepresentation>(
				   target, "SurgSim::Graphics::PointCloudRepresentation");
}

std::shared_ptr<SurgSim::Physics::DeformableRepresentation> TransferPhysicsToPointCloudBehavior::getSource() const
{
	return m_source;
}

std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation> TransferPhysicsToPointCloudBehavior::getTarget() const
{
	return m_target;
}

void TransferPhysicsToPointCloudBehavior::update(double dt)
{
	auto state = m_source->getFinalState();

	auto target = m_target->getVertices();
	for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
	{
		target->setVertexPosition(nodeId, state->getPosition(nodeId));
	}
}

bool TransferPhysicsToPointCloudBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToPointCloudBehavior::doWakeUp()
{
	auto state = m_source->getFinalState();
	auto target = m_target->getVertices();

	if (target->getNumVertices() == 0)
	{
		for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
		{
			SurgSim::Graphics::PointCloud::VertexType vertex(state->getPosition(nodeId));
			target->addVertex(vertex);
		}
	}
	return true;
}

}; //namespace Blocks
}; //namespace SurgSim
