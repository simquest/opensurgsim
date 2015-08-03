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

#include "SurgSim/Blocks/TransferPhysicsToCurveBehavior.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Graphics/CurveRepresentation.h"

namespace SurgSim
{

namespace Blocks
{


TransferPhysicsToCurveBehavior::TransferPhysicsToCurveBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{

}


void TransferPhysicsToCurveBehavior::update(double dt)
{
	auto state = m_source->getFinalState();
	for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
	{
		m_vertices.setVertexPosition(nodeId, state->getPosition(nodeId));
	}
	m_target->updateControlPoints(m_vertices);
}

bool TransferPhysicsToCurveBehavior::doInitialize()
{
	return true;
}

bool TransferPhysicsToCurveBehavior::doWakeUp()
{
	auto state = m_source->getFinalState();

	if (m_vertices.getNumVertices() == 0)
	{
		for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
		{
			SurgSim::Graphics::CurveRepresentation::ControlPointType::VertexType vertex(state->getPosition(nodeId));
			m_vertices.addVertex(vertex);
		}
	}
	return true;
}

}
}
