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

#ifndef SURGSIM_DATASTRUCTURES_AABBTREE_H
#define SURGSIM_DATASTRUCTURES_AABBTREE_H

#include <list>
#include <vector>

#include "SurgSim/DataStructures/Tree.h"
#include "SurgSim/DataStructures/AabbTreeData.h"
#include "SurgSim/Math/Aabb.h"

namespace SurgSim
{

namespace DataStructures
{

class AabbTreeNode;

/// AabbTree is a tree that is organized by the bounding boxes of the referenced objects, the bounding box used is
/// the Axis Aligned Bounding Box (AABB), with the extents of an AABB describing the min and max of each coordinate
/// for the given object.
class AabbTree : public Tree
{
public:

	/// Constructor
	AabbTree();

	/// Constructor
	/// \param maxObjectsPerNode if the number of objects exceeds this a split of the node will be triggered
	explicit AabbTree(size_t maxObjectsPerNode);

	/// Destructor
	virtual ~AabbTree();

	/// \return the number of objects per node that will trigger a split for this tree
	size_t getMaxObjectsPerNode() const;

	/// Add a give object identified by objectId to the tree, this id should be unqiue on the users side, but no
	/// checks are made in the inside of the tree
	/// \param aabb AABB of this object.
	/// \param objectId Id for the object to be identified with this bounding box
	void add(const SurgSim::Math::Aabbd& aabb, size_t objectId);

	/// Create the tree from a list of tree items, all the tree information will be deleted
	/// \param items list of items to insert into the tree
	void set(const AabbTreeData::ItemList& items);

	/// Create the tree from a list of tree items, all the tree information will be deleted
	/// \param items rvalue reference to list of items to insert into the tree
	void set(AabbTreeData::ItemList&& items);

	/// \return the AABB for the tree
	const SurgSim::Math::Aabbd& getAabb() const;

	/// Type indicating a relationship between two AabbTreeNodes
	typedef std::pair<AabbTreeNode*, AabbTreeNode*> TreeNodePairType;

	/// Query to find all pairs of intersecting nodes between two aabb r-trees.
	/// \param otherTree The other tree to compare against
	/// \return The list of all pairs of intersecting nodes
	std::vector<TreeNodePairType> spatialJoin(const AabbTree& otherTree) const;

	/// Query to find all pairs of intersecting nodes between two aabb r-trees.
	/// \param lhsParent root node of the first tree
	/// \param rhsParent root node of the second tree
	/// \param result the list of all pairs of intersecting nodes
	void spatialJoin(AabbTreeNode* lhsParent, AabbTreeNode* rhsParent,
		std::vector<TreeNodePairType>* result) const;

	void updateBounds(const std::vector<Math::Aabbd>& bounds);

	void updateNodeBounds(const std::vector<Math::Aabbd>& bounds, SurgSim::DataStructures::AabbTreeNode* node);

private:

	/// Number of objects in a node that will trigger a split
	size_t m_maxObjectsPerNode;

	/// A typed version of the root for access without typecasting
	std::shared_ptr<AabbTreeNode> m_typedRoot;
};

}
}

#endif
