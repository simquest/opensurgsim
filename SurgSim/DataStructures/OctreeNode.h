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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Aabb.h"

namespace SurgSim
{

namespace Math
{
class OctreeShape;
}

namespace DataStructures
{

/// Typedef of octree path
/// The path is a vector of children indexes (each within 0 to 7) that lead to
/// the specific node the front of the vector holds the index of the root's children.
typedef std::vector<size_t> OctreePath;

enum Neighborhood
{
	NEIGHBORHOOD_NONE = 0x0,
	NEIGHBORHOOD_FACE = 0x1,
	NEIGHBORHOOD_EDGE = 0x2,
	NEIGHBORHOOD_VERTEX  = 0x4
};

enum Symbol
{
	SYMBOL_HALT = -1,
	SYMBOL_DOWN = 0,
	SYMBOL_UP = 1,
	SYMBOL_RIGHT = 2,
	SYMBOL_LEFT = 3,
	SYMBOL_BACK = 4,
	SYMBOL_FRONT = 5
};

/// Calculate the neighbor of an node in the octree by traversing a state machine, see
/// http://ww1.ucmss.com/books/LFS/CSREA2006/MSV4517.pdf for detailed description.
/// The information about the location of a nodes neighbor is encoded in a state machine, this machine is traversed
/// until a 'Halt' instruction is found. If the neighbor is across a boundary on the octree, a new search direction
/// is determined by the algorithm.
/// \param source the node whose neighbor is needed
/// \param direction a set of directions, for face neighbors use 1, for edge neigbhors use 2 and for vertex neighbors
///        use 3 direction, fill the other spots with SYMBOL_HALT. E.g. to find the left neighbor use
///        {SYMBOL_LEFT, SYMBOL_HALT, SYMBOL_HALT}, for the vertex neighber on the upper left front corner use
///        {SYMBOL_LEFT, SYMBOLD_FRONT, SYMBOL_UP}
OctreePath getNeighbor(const OctreePath& source, const std::array<int, 3>& direction);


std::vector<OctreePath> getNeighbors(const OctreePath& source, Neighborhood type);

/// Octree data structure
///
/// The octree node consists of an axis aligned bounding box, that can be
/// subdivided into 8 equally sized subregions. Each subregion is an octree
/// node and can be further subdivided.
/// with x-right and y-up on a right handed coordinate system this is the ordering of the nodes of the tree, looking
/// down the z-axis
/// Back Face
///        2  3
///        0  1
/// Front Face
///	       6  7
///        4  5
///
/// \tparam	Data Type of extra data stored in each node
template<class Data>
class OctreeNode : public SurgSim::Framework::Asset,
	public std::enable_shared_from_this<OctreeNode<Data>>
{
	friend class SurgSim::Math::OctreeShape;

public:

	/// Bounding box type for convenience
	typedef Eigen::AlignedBox<double, 3> AxisAlignedBoundingBox;

	/// Constructor
	OctreeNode();

	/// Copy constructor when the template data is the same type
	/// \param other the octree to copy from
	OctreeNode(const OctreeNode& other);

	/// Copy constructor when the template data is a different type
	/// In this case, no data will be copied
	/// \tparam T type of data stored in the other node
	/// \param other the octree to copy from
	template <class T>
	OctreeNode(const OctreeNode<T>& other);

	/// Constructor
	/// \param  boundingBox The region contained by this octree node
	explicit OctreeNode(const SurgSim::Math::Aabbd& boundingBox);

	/// Destructor
	virtual ~OctreeNode();

	/// Get the bounding box for this octree node
	/// \return the bounding box
	const SurgSim::Math::Aabbd& getBoundingBox() const;

	/// Is this node active
	/// \return true if the octree node is active
	bool isActive() const;

	/// Set active flag for this octree node
	/// \param isActive True if the octree node is being activated, False otherwise
	void setIsActive(bool isActive);

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
	/// \return true if data is added
	bool addData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level);

	/// Get the children of this node (non const version)
	/// \return vector of all eight children
	std::array<std::shared_ptr<OctreeNode<Data> >, 8>& getChildren();

	/// Get the children of this node
	/// \return vector of all eight children
	const std::array<std::shared_ptr<OctreeNode<Data> >, 8>& getChildren() const;

	/// Get a child of this node (non const version)
	/// \param index the child index
	/// \return the requested octree node
	/// NOTE: an exception will be thrown if the index >= 8
	std::shared_ptr<OctreeNode<Data> > getChild(size_t index);

	/// Get a child of this node
	/// \param index the child index
	/// \return the requested octree node
	/// NOTE: an exception will be thrown if the index >= 8
	const std::shared_ptr<OctreeNode<Data> > getChild(size_t index) const;

	/// Get the node at the supplied path
	/// \param path the path to the specific node
	/// \return the requested octree node
	virtual std::shared_ptr<OctreeNode<Data> > getNode(const OctreePath& path);

	/// Extra node data
	Data data;

protected:
	/// Recursive function that does the adding of the data to the octree
	/// \param position The position to add the data at
	/// \param nodeData The data to store in the node
	/// \param level The number of levels down the octree to store the data
	/// \param currentLevel Used to keep track of the current level during recursive calls
	/// \return true if data is added
	bool doAddData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level,
				   const int currentLevel);

	virtual bool doLoad(const std::string& filePath) override;

	/// The bounding box of the current OctreeNode
	SurgSim::Math::Aabbd m_boundingBox;

	/// True if there is any data inside this node, including data held by children, children's children, etc.
	bool m_isActive;

	/// True if the node has children
	bool m_hasChildren;

	/// The children of this node
	std::array<std::shared_ptr<OctreeNode<Data> >, 8> m_children;
};


/// A free function to load an octree from file.
/// \param fileName	Name of the external file which contains an octree.
/// \return A std::shared_ptr<> pointing to an OctreeNode.
std::shared_ptr<OctreeNode<SurgSim::DataStructures::EmptyData>> loadOctree(const std::string& fileName);

};  // namespace DataStructures
};  // namespace SurgSim

#include "SurgSim/DataStructures/OctreeNode-inl.h"

#endif // SURGSIM_DATASTRUCTURES_OCTREENODE_H
