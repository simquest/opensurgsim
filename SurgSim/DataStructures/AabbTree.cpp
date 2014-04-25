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

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"

#include <memory>

namespace SurgSim
{
namespace DataStructures
{

AabbTree::AabbTree() :
	m_maxObjectsPerNode(3)
{
	m_typedRoot = std::make_shared<AabbTreeNode>();
	setRoot(m_typedRoot);
}

AabbTree::AabbTree(size_t maxObjectsPerNode) :
	m_maxObjectsPerNode(maxObjectsPerNode)
{
	m_typedRoot = std::make_shared<AabbTreeNode>();
	setRoot(m_typedRoot);
}

AabbTree::~AabbTree()
{

}

void AabbTree::add(const SurgSim::Math::Aabbd& aabb, size_t objectId)
{
	m_typedRoot->addData(aabb, objectId, m_maxObjectsPerNode);
}

size_t AabbTree::getMaxObjectsPerNode() const
{
	return m_maxObjectsPerNode;
}

const SurgSim::Math::Aabbd& AabbTree::getAabb() const
{
	return m_typedRoot->getAabb();
}

}
}

