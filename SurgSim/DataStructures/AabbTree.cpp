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

void AabbTree::set(const AabbTreeData::ItemList& items)
{
	m_typedRoot = std::make_shared<AabbTreeNode>();
	setRoot(m_typedRoot);
	m_typedRoot->setData(items, m_maxObjectsPerNode);
}

void AabbTree::set(AabbTreeData::ItemList&& items)
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

std::vector<AabbTree::TreeNodePairType> AabbTree::spatialJoin(const AabbTree& otherTree) const
{
	std::vector<TreeNodePairType> result;

	spatialJoin(static_cast<AabbTreeNode*>(getRoot().get()),
				static_cast<AabbTreeNode*>(otherTree.getRoot().get()),
				&result);

	return result;
}

void AabbTree::spatialJoin(AabbTreeNode* lhsParent, AabbTreeNode* rhsParent,
	std::vector<TreeNodePairType>* result) const
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
			auto rhs = static_cast<AabbTreeNode*>(rhsParent->getChild(j).get());
			spatialJoin(lhsParent, rhs, result);
		}
	}
	else if (rhsParent->getNumChildren() == 0)
	{
		for (size_t i = 0; i < lhsParent->getNumChildren(); i++)
		{
			auto lhs = static_cast<AabbTreeNode*>(lhsParent->getChild(i).get());
			spatialJoin(lhs, rhsParent, result);
		}
	}
	else
	{
		for (size_t i = 0; i < lhsParent->getNumChildren(); i++)
		{
			auto lhs = static_cast<AabbTreeNode*>(lhsParent->getChild(i).get());

			for (size_t j = 0; j < rhsParent->getNumChildren(); j++)
			{
				auto rhs = static_cast<AabbTreeNode*>(rhsParent->getChild(j).get());
				spatialJoin(lhs, rhs, result);
			}
		}
	}
}

void AabbTree::updateBounds(const std::vector<Math::Aabbd>& bounds)
{
	updateNodeBounds(bounds, static_cast<SurgSim::DataStructures::AabbTreeNode*>(getRoot().get()));
}

void AabbTree::updateNodeBounds(const std::vector<Math::Aabbd>& bounds,
								SurgSim::DataStructures::AabbTreeNode* node)
{
	const size_t numChildren = node->getNumChildren();
	if (numChildren > 0)
	{
		auto* child = static_cast<SurgSim::DataStructures::AabbTreeNode*>(node->getChild(0).get());
		updateNodeBounds(bounds, child);
		auto aabb = child->getAabb();

		for (size_t i = 1; i < numChildren; ++i)
		{
			auto* child = static_cast<SurgSim::DataStructures::AabbTreeNode*>(node->getChild(i).get());
			updateNodeBounds(bounds, child);
			aabb.extend(child->getAabb());
		}
		node->setAabb(aabb);
	}
	else
	{
		auto data = static_cast<SurgSim::DataStructures::AabbTreeData*>(node->getData().get());
		for (auto& item : data->getData())
		{
			item.first = bounds[item.second];
		}
		data->recalculateAabb();
		node->setAabb(data->getAabb());
	}
}

}
}

