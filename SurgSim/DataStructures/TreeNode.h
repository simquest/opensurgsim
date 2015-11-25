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

#ifndef SURGSIM_DATASTRUCTURES_TREENODE_H
#define SURGSIM_DATASTRUCTURES_TREENODE_H

#include <vector>
#include <memory>

#include "SurgSim/DataStructures/TreeVisitor.h"

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
	/// \param data The data for this node.
	void setData(std::shared_ptr<TreeData> data);

	/// \return The data of this node.
	std::shared_ptr<TreeData> getData() const;

	/// \return The number of children of this node.
	size_t getNumChildren() const;

	/// Returns the specified child of this node.
	/// \param index	Index of the child
	/// \return Child at the specified index
	std::shared_ptr<TreeNode> getChild(size_t index) const;

	/// Public entry point for visitor, currently this performs pre-order traversal of the tree
	/// \param visitor The visitor that wants to traverse the tree
	void accept(TreeVisitor* visitor);

	/// Returns true if the nodes are equal; otherwise, returns false.
	/// If the nodes are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const TreeNode&).
	/// \param node The node for comparison.
	/// \return true is this node and the one from the parameter are equal.
	bool operator==(const TreeNode& node) const;

	/// Returns true if the nodes are not equal; otherwise, returns false.
	/// If the nodes are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const TreeNode&).
	/// \param node The node for comparison.
	/// \return true if this node and the parameter are not equal
	bool operator!=(const TreeNode& node) const;

	void clear();

protected:

	/// Returns true if the nodes are equal; otherwise, returns false.
	/// Recurses on children.
	/// Override this method in derived classes to implement different comparisons.
	/// \param node The node for comparison.
	/// \return true if this node is equal to the node in the parameter.
	virtual bool isEqual(const TreeNode& node) const;

	/// Private function for use with the visitor pattern, this needs to be implemented
	/// to make the correct double dispatch call to the dynamic type of this class.
	/// \param visitor The visitor that is trying to traverse the tree.
	/// \return true to indicate proceeding with the visitor, false indicates to abort the traversal.
	virtual bool doAccept(TreeVisitor* visitor) = 0;

	/// Sets the number of children of this node.
	/// Any added children will be null.
	/// \param numChildren The new number of children.
	void setNumChildren(size_t numChildren);

	/// Add a child to this node.
	/// \param node The new child node.
	void addChild(const std::shared_ptr<TreeNode>& node);

	/// Add a child to this node.
	/// \param node The new child node.
	void addChild(const std::shared_ptr<TreeNode>&& node);

	/// Set a specific child of this node.
	/// \param index	Index of the child
	/// \param node		Node to become a child
	void setChild(size_t index, const std::shared_ptr<TreeNode>& node);

private:

	/// Children of this node.
	std::vector<std::shared_ptr<TreeNode>> m_children;

	/// Data of this node.
	std::shared_ptr<TreeData> m_data;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TREENODE_H
