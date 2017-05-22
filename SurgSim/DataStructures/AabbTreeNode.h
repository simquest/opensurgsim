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

#ifndef SURGSIM_DATASTRUCTURES_AABBTREENODE_H
#define SURGSIM_DATASTRUCTURES_AABBTREENODE_H

#include "SurgSim/DataStructures/TreeNode.h"
#include "SurgSim/DataStructures/AabbTreeData.h"

#include "SurgSim/Math/Aabb.h"

namespace SurgSim
{
namespace DataStructures
{

/// Node class for the AabbTree, this handles groups of items and subdivision if the number of items gets too big
class AabbTreeNode : public TreeNode
{
public:

	using TreeNode::setData;

	/// Constructor
	AabbTreeNode();

	/// Destructor
	virtual ~AabbTreeNode();

	/// Splits the data into two parts, creates two children and puts the split data into the children
	/// the aabb of this node does not change, the data of this node will be empty after this.
	/// \param maxNodeData number of maximum items of data in this node, if more, the node will split,
	///					   if 0 the node will not be split until it is no longer possible, the structure will
	///                    approach a binary tree.
	/// \note Sometimes the current mechanism to split the list of AABBs along the longest axis will fail to actually
	///       split the list, if the size of the list is greater than 3 * maxNodeData a warning will be generated
	void splitNode(size_t maxNodeData = 0);

	/// Get the aabb of this node, it is the union of the aabb of all the items in the data when the node
	/// has data, or all the union of the aabb trees of all the sub-nodes.
	/// \return The aabb box of this node.
	const SurgSim::Math::Aabbd& getAabb() const;

	void setAabb(const SurgSim::Math::Aabbd& aabb);

	/// Add data to this node, if maxNodeData is >0 the node will split if the number of data items exceeds maxNodeData
	/// \param aabb The aabb for the item to be added.
	/// \param id The id for the item that is being added, handled by the user of this class.
	/// \param maxNodeData number of maximum items of data in this node, if more, the node will split,
	///					   if 0 the node will not be split until it is no longer possible, the structure will
	///                    approach a binary tree.
	void addData(const SurgSim::Math::Aabbd& aabb, size_t id, size_t maxNodeData = 0);

	/// Set the data on this node, the node needs to be empty and not have any children for this to work.
	/// Nodes will be split until there are only maxNodeData elements in one node
	/// \param items list of AabbTreeData::Item elements that should populate the tree
	/// \param maxNodeData number of maximum items of data in this node, if more, the node will split,
	///					   if 0 the node will not be split until it is no longer possible, the structure will
	///                    approach a binary tree.
	void setData(const AabbTreeData::ItemList& items, size_t maxNodeData = 0);

	/// Set the data on this node, rvalue reference version,
	/// the node needs to be empty and not have any children for this to work.
	/// Nodes will be split until there are only maxNodeData elements in one node
	/// \param items list of AabbTreeData::Item elements that should populate the tree
	/// \param maxNodeData number of maximum items of data in this node, if more, the node will split,
	///					   if 0 the node will not be split until it is no longer possible, the structure will
	///                    approach a binary tree.
	void setData(AabbTreeData::ItemList&& items, size_t maxNodeData);

	/// Fetch a list of items that have AABBs intersecting with the given AABB.
	/// \param aabb The bounding box for the query.
	/// \param [out] result location to receive the results of the call.
	void getIntersections(const SurgSim::Math::Aabbd& aabb, std::vector<size_t>* result);

protected:

	bool doAccept(TreeVisitor* visitor) override;

private:

	/// The internal bounding box for this node, it is used when the node does not have any data
	SurgSim::Math::Aabbd m_aabb;

	/// Cache for the index of the longest axis on this node
	size_t m_axis;
};

}
}

#endif
