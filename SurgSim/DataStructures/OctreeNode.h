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

#ifndef SURGSIM_DATASTRUCTURES_OCTREENODE_H
#define SURGSIM_DATASTRUCTURES_OCTREENODE_H

#include <array>
#include <memory>
#include <Eigen/Geometry>

#include <SurgSim/Math/Vector.h>


namespace SurgSim
{

namespace DataStructures
{


/// Octree data structure
///
/// The octree node consists of an axis aligned bounding box, that can be
/// subdivided into 8 equally sized subregions. Each subregion is an octree
/// node and can be further subdivided.
///
/// \tparam	Data Type of extra data stored in each node
template<class Data>
class OctreeNode
{
public:
	/// Bounding box type for convenience
	typedef Eigen::AlignedBox<double, 3> BoundingBoxType;

	/// Constructor
	/// \param  boundingBox The region contained by this octree node
	explicit OctreeNode(const BoundingBoxType& boundingBox);

	/// Destructor
	virtual ~OctreeNode();

	/// Get the bounding box for this octree node
	/// \return the bounding box
	const BoundingBoxType& getBoundingBox() const;

	/// Is this node active
	/// \return true if there is any data inside this node, including data held 
	/// by children, children's children, etc.
	bool isActive() const;

	/// Does this node have children
	/// \return true if this node has children
	bool hasChildren() const;

	/// Subdivide the node into 8 equal regions. Each subregion will be stored
	/// as this nodes children.
	/// NOTE: The data stored in the current node will not be automatically subdivided.
	void subdivide();

	/// Add data to a node in this octree
	/// The octree will build the octree as necessary to add the
	/// node at the specified level
	/// \param position The position to add the data at
	/// \param nodeData The data to store in the node
	/// \param level The number of levels down the octree to store the data
	bool addData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level);

	/// Get the children of this node (non const version)
	/// \return vector of all eight children
	std::array<std::shared_ptr<OctreeNode<Data> >, 8>& getChildren();

	/// Get the children of this node
	/// \return vector of all eight children
	const std::array<std::shared_ptr<OctreeNode<Data> >, 8>& getChildren() const;

	/// Extra node data
	Data data;

private:
	bool doAddData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level,
			const int currentLevel);

	BoundingBoxType m_boundingBox;
	bool m_isActive;
	bool m_hasChildren;
	std::array<std::shared_ptr<OctreeNode<Data> >, 8> m_children;
};

};  // namespace DataStructures
};  // namespace SurgSim

#include <SurgSim/DataStructures/OctreeNode-inl.h>

#endif // SURGSIM_DATASTRUCTURES_OCTREENODE_H
