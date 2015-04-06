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

void AabbTree::set(const std::list<AabbTreeData::Item>& items)
{
	m_typedRoot = std::make_shared<AabbTreeNode>();
	setRoot(m_typedRoot);
	m_typedRoot->setData(items, m_maxObjectsPerNode);
}

void AabbTree::set(std::list<AabbTreeData::Item>&& items)
{
	m_typedRoot = std::make_shared<AabbTreeNode>();
	setRoot(m_typedRoot);
	m_typedRoot->setData(std::move(items), m_maxObjectsPerNode);
}

size_t AabbTree::getMaxObjectsPerNode() const
{
	return m_maxObjectsPerNode;
}

const SurgSim::Math::Aabbd& AabbTree::getAabb() const
{
	return m_typedRoot->getAabb();
}

std::list<AabbTree::TreeNodePairType> AabbTree::spatialJoin(const AabbTree& otherTree) const
{
	std::list<TreeNodePairType> result;

	spatialJoin(std::static_pointer_cast<AabbTreeNode>(getRoot()),
				std::static_pointer_cast<AabbTreeNode>(otherTree.getRoot()),
				&result);

	return result;
}

void AabbTree::spatialJoin(std::shared_ptr<AabbTreeNode> lhsParent,
						   std::shared_ptr<AabbTreeNode> rhsParent,
						   std::list<TreeNodePairType>* result) const
{
	if (!SurgSim::Math::doAabbIntersect(lhsParent->getAabb(), rhsParent->getAabb()))
	{
		return;
	}

	if ((lhsParent->getNumChildren() == 0) && (rhsParent->getNumChildren() == 0))
	{
		result->emplace_back(lhsParent, rhsParent);
	}
	else if (lhsParent->getNumChildren() == 0)
	{
		for (size_t j = 0; j < rhsParent->getNumChildren(); j++)
		{
			auto rhs = std::static_pointer_cast<AabbTreeNode>(rhsParent->getChild(j));
			spatialJoin(lhsParent, rhs, result);
		}
	}
	else if (rhsParent->getNumChildren() == 0)
	{
		for (size_t i = 0; i < lhsParent->getNumChildren(); i++)
		{
			auto lhs = std::static_pointer_cast<AabbTreeNode>(lhsParent->getChild(i));
			spatialJoin(lhs, rhsParent, result);
		}
	}
	else
	{
		for (size_t i = 0; i < lhsParent->getNumChildren(); i++)
		{
			auto lhs = std::static_pointer_cast<AabbTreeNode>(lhsParent->getChild(i));

			for (size_t j = 0; j < rhsParent->getNumChildren(); j++)
			{
				auto rhs = std::static_pointer_cast<AabbTreeNode>(rhsParent->getChild(j));
				spatialJoin(lhs, rhs, result);
			}
		}
	}
}

}
}

