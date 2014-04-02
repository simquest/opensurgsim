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
#include <memory>

namespace SurgSim
{

namespace DataStructures
{

class TreeData;


/// Basic tree node structure.
/// The nodes build up the structure of a Tree.
/// \sa Tree TreeData
class TreeNode
{
public:
	/// Constructor. After construction, the node has no children, and the data is null.
	TreeNode();
	/// Destructor
	virtual ~TreeNode();

	/// Sets the data of this node.
	void setData(std::shared_ptr<TreeData> data)
	{
		m_data = data;
	}
	/// Returns the data of this node.
	std::shared_ptr<TreeData> getData() const
	{
		return m_data;
	}

	/// Sets the number of children of this node.
	/// Any added children will be null.
	void setNumChildren(unsigned int numChildren)
	{
		m_children.resize(numChildren);
	}
	/// Returns the number of children of this node.
	unsigned int getNumChildren() const
	{
		return m_children.size();
	}

	/// Add a child to this node.
	void addChild(std::shared_ptr<TreeNode>& node)
	{
		m_children.push_back(node);
	}
	/// Set a specific child of this node.
	/// \param index	Index of the child
	/// \param node		Node to become a child
	void setChild(unsigned int index, std::shared_ptr<TreeNode>& node)
	{
		m_children[index] = node;
	}
	/// Returns the specified child of this node.
	/// \param index	Index of the child
	/// \return Child at the specified index
	std::shared_ptr<TreeNode> getChild(unsigned int index) const
	{
		return m_children[index];
	}

	/// Returns true if the nodes are equal; otherwise, returns false.
	/// If the nodes are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const TreeNode&).
	bool operator==(const TreeNode& node) const;

	/// Returns true if the nodes are not equal; otherwise, returns false.
	/// If the nodes are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const TreeNode&).
	bool operator!=(const TreeNode& node) const;

protected:
	/// Returns true if the nodes are equal; otherwise, returns false.
	/// Recurses on children.
	/// Override this method in derived classes to implement different comparisons.
	virtual bool isEqual(const TreeNode& node) const;

private:
	/// Children of this node.
	std::vector<std::shared_ptr<TreeNode>> m_children;

	/// Data of this node.
	std::shared_ptr<TreeData> m_data;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_TREE_NODE_H
