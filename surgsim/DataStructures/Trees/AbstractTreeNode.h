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

#ifndef SURGSIM_TREE_NODE_H
#define SURGSIM_TREE_NODE_H

#include "TreeData.h"

namespace SurgSim
{

class TreeNode
{
public:
	virtual ~TreeNode()
	{
	}

	virtual bool operator==(const TreeNode& node) const
	{
		return (typeid(*this) == typeid(node)) && isEqual(node);
	}

	virtual bool operator!=(const TreeNode& node) const
	{
		return (typeid(*this) != typeid(node)) || ! isEqual(node);
	}

	virtual void setData(std::shared_ptr<TreeData> data)
	{
		m_data = data;
	}
	std::shared_ptr<TreeData> getData() const
	{
		return m_data;
	}

	virtual void setNumChildren(unsigned int numChildren)
	{
		m_children.resize(numChildren);
	}
	unsigned int getNumChildren() const
	{
		return m_children.size();
	}

	virtual void addChild(std::shared_ptr<TreeNode>& node)
	{
		m_children.push_back(node);
	}
	virtual void setChild(unsigned int index, std::shared_ptr<TreeNode>& node)
	{
		m_children[index] = node;
	}
	std::shared_ptr<TreeNode> getChild(unsigned int index) const
	{
		return m_children[index];
	}

protected:
	TreeNode() : m_data(nullptr) 
	{
	}

	virtual bool isEqual(const TreeNode& node) const
	{
		if ( (m_data != nullptr && node.m_data == nullptr) || (m_data == nullptr && node.m_data != nullptr) )
		{
			return false;
		}
		if (m_data != nullptr && node.m_data != nullptr)
		{
			return *m_data == *node.m_data;
		}
		return true;
	}

private:
	std::vector< std::shared_ptr<TreeNode> > m_children;

	std::shared_ptr<TreeData> m_data;
};

};  // namespace SurgSim

#endif  // TREE_NODE_H
