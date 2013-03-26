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

#ifndef SURGSIM_DATA_STRUCTURES_TREE_NODE_H
#define SURGSIM_DATA_STRUCTURES_TREE_NODE_H

#include <vector>

namespace SurgSim
{

namespace DataStructures
{

class TreeData;

class TreeNode
{
public:
	TreeNode();
	virtual ~TreeNode();

	void setData(std::shared_ptr<TreeData> data)
	{
		m_data = data;
	}
	std::shared_ptr<TreeData> getData() const
	{
		return m_data;
	}

	void setNumChildren(unsigned int numChildren)
	{
		m_children.resize(numChildren);
	}
	unsigned int getNumChildren() const
	{
		return m_children.size();
	}

	void addChild(std::shared_ptr<TreeNode>& node)
	{
		m_children.push_back(node);
	}
	void setChild(unsigned int index, std::shared_ptr<TreeNode>& node)
	{
		m_children[index] = node;
	}
	std::shared_ptr<TreeNode> getChild(unsigned int index) const
	{
		return m_children[index];
	}

	bool operator==(const TreeNode& node) const;

	bool operator!=(const TreeNode& node) const;

protected:
	virtual bool isEqual(const TreeNode& node) const;

private:
	std::vector< std::shared_ptr<TreeNode> > m_children;

	std::shared_ptr<TreeData> m_data;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_TREE_NODE_H
