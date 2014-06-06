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

#ifndef SURGSIM_BLOCKS_TRANSFERODESTATETOVERTICESBEHAVIOR_INL_H
#define SURGSIM_BLOCKS_TRANSFERODESTATETOVERTICESBEHAVIOR_INL_H

#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Blocks
{

template <class VertexData>
TransferOdeStateToVerticesBehavior<VertexData>::TransferOdeStateToVerticesBehavior(
	const std::string& name,
	std::shared_ptr<SurgSim::Math::OdeState> from,
	std::shared_ptr<SurgSim::DataStructures::Vertices<VertexData>> to) :
	SurgSim::Framework::Behavior(name),
	m_from(from),
	m_to(to)
{
}

template <class VertexData>
void TransferOdeStateToVerticesBehavior<VertexData>::update(double dt)
{
	// Note that transfer is called without initialization on
	// This should have been done in doWakeUp already
	transfer();
}

template <class VertexData>
bool TransferOdeStateToVerticesBehavior<VertexData>::doInitialize()
{
	return true;
}

template <class VertexData>
bool TransferOdeStateToVerticesBehavior<VertexData>::doWakeUp()
{
	// Note that transfer is called with initialization on
	// This is done in doWakeUp as an external data structure will be initialized (Vertices)
	transfer(true);
	return true;
}

template <class VertexData>
void TransferOdeStateToVerticesBehavior<VertexData>::transfer(bool doInitialization)
{
	const size_t numNodes = m_from->getNumNodes();

	// If initialization is requested and vertices is empty, let's populate it properly
	if (doInitialization == true && m_to->getNumVertices() == 0 && numNodes != 0)
	{
		for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
		{
			SurgSim::DataStructures::Vertex<VertexData> v(m_from->getPosition(nodeId));
			m_to->addVertex(v);
		}
	}
	else if (m_to->getNumVertices() == numNodes)
	{
		for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
		{
			m_to->setVertexPosition(nodeId, m_from->getPosition(nodeId));
		}
	}
}

}; //namespace Blocks

}; //namespace SurgSim

#endif // SURGSIM_BLOCKS_TRANSFERODESTATETOVERTICESBEHAVIOR_INL_H
