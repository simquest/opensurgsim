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

#include "SurgSim/DataStructures/Tree.h"

#include "SurgSim/Math/Aabb.h"

#include <vector>

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
	/// \param maxObjectsPerNode, if the number of objects exceeds this a split of the node will be triggered
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

	SurgSim::Math::Aabbd getAabb();

private:

	/// Number of objects in a node that will trigger a split
	size_t m_maxObjectsPerNode;

	/// A typed version of the root for access without typecasting
	std::shared_ptr<AabbTreeNode> m_typedRoot;
};

}
}

#endif
