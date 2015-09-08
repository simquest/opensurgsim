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

#include "SurgSim/Physics/Spring.h"

namespace SurgSim
{
namespace Physics
{

Spring::~Spring()
{}

void Spring::initialize(const SurgSim::Math::OdeState& state)
{
}

size_t Spring::getNumNodes() const
{
	return m_nodeIds.size();
}

size_t Spring::getNodeId(size_t springNodeId) const
{
	return m_nodeIds[springNodeId];
}

const std::vector<size_t>& Spring::getNodeIds() const
{
	return m_nodeIds;
}

} // namespace Physics
} // namespace SurgSim
