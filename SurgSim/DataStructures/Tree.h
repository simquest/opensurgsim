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

#ifndef SURGSIM_DATASTRUCTURES_TREE_H
#define SURGSIM_DATASTRUCTURES_TREE_H

#include <memory>

namespace SurgSim
{

namespace DataStructures
{

class TreeNode;

/// Basic tree structure.
/// The tree is composed of TreeNodes, which are all accessible from the root.
/// \sa TreeNode TreeData
class Tree
{
public:
	/// Constructor. After construction, the root is null.
	Tree();
	/// Destructor
	virtual ~Tree();

	/// Sets the root of the tree.
	void setRoot(std::shared_ptr<TreeNode> root)
	{
		m_root = root;
	}
	/// Gets the root of the tree.
	std::shared_ptr<TreeNode> getRoot() const
	{
		return m_root;
	}

	/// Returns true if the trees are equal; otherwise, returns false.
	/// If the trees are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const Tree&).
	bool operator==(const Tree& tree) const;

	/// Returns true if the trees are not equal; otherwise, returns false.
	/// If the trees are not of the same type, returns false;
	/// otherwise, compares with the implementation of isEqual(const Tree&).
	bool operator!=(const Tree& tree) const;

protected:
	/// Returns true if the trees are equal; otherwise, returns false.
	/// Recurses through the tree, starting at the root.
	/// Override this method in derived classes to implement different comparisons.
	virtual bool isEqual(const Tree& tree) const;

private:
	/// Root of the tree.
	std::shared_ptr<TreeNode> m_root;

};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_TREE_H
