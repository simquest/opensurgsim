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

#include "SurgSim/DataStructures/TreeNode.h"

#include "SurgSim/DataStructures/TreeData.h"

#include <typeinfo>

namespace SurgSim
{
namespace DataStructures
{
TreeNode::TreeNode() : m_data(nullptr)
{
}
TreeNode::~TreeNode()
{
}

bool TreeNode::operator==(const TreeNode& node) const
{
	return (typeid(*this) == typeid(node)) && isEqual(node);
}

bool TreeNode::operator!=(const TreeNode& node) const
{
	return (typeid(*this) != typeid(node)) || ! isEqual(node);
}

bool TreeNode::isEqual(const TreeNode& node) const
{
	if ((m_data != nullptr && node.m_data == nullptr) || (m_data == nullptr && node.m_data != nullptr))
	{
		return false;
	}
	if (m_data != nullptr && node.m_data != nullptr)
	{
		return *m_data == *node.m_data;
	}
	return true;
}

void TreeNode::setData(std::shared_ptr<TreeData> data)
{
	m_data = data;
}

std::shared_ptr<TreeData> TreeNode::getData() const
{
	return m_data;
}

void TreeNode::setNumChildren(unsigned int numChildren)
{
	m_children.resize(numChildren);
}

unsigned int TreeNode::getNumChildren() const
{
	return m_children.size();
}

void TreeNode::addChild(const std::shared_ptr<TreeNode>& node)
{
	m_children.push_back(node);
}

void TreeNode::addChild(const std::shared_ptr<TreeNode>&& node)
{
	m_children.push_back(node);
}

void TreeNode::setChild(unsigned int index, const std::shared_ptr<TreeNode>& node)
{
	m_children[index] = node;
}

std::shared_ptr<TreeNode> TreeNode::getChild(unsigned int index) const
{
	return m_children[index];
}

void TreeNode::accept(TreeVisitor* visitor)
{
	if (doAccept(visitor))
	{
		for (size_t i = 0; i < getNumChildren(); ++i)
		{
			getChild(i)->accept(visitor);
		}
	}
}

}
}

