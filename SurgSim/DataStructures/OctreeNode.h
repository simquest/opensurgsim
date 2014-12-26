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
#include <functional>

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

/// Enable the OctreePath to be used as a key in an unordered map, if the int range is exceeded this will just
/// push the least significant numbers (root addresses) out of scope, it loses a little bit of address space as
/// octree ids only go from 0-7
class OctreePathHash
{
public:
	size_t operator()(const OctreePath& path) const
	{
		size_t result = 0;
		for (auto i : path)
		{
			result = (result << 3) | i;
		}

		return m_hasher(result);
	}
private:
	std::hash<size_t> m_hasher;
};

/// Indicates what neighbors to grab
enum Neighborhood
{
	NEIGHBORHOOD_NONE = 0x0,
	NEIGHBORHOOD_FACE = 0x1,
	NEIGHBORHOOD_EDGE = 0x2,
	NEIGHBORHOOD_VERTEX  = 0x4,
	NEIGHBORHOOD_ALL = 0x1 | 0x2 | 0x4
};

/// Direction code for the neighborhood search
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
/// \note The numbering in the paper and our numbering is slightly different, this means the following transformations
///		  took place.
///       a) The colums where reordered to match our numbering
///       b) All the numbers where replaced to match out numbering
///       The number changes where as following: 0 => 6, 1 => 7, 2 => 4, 3 => 5, 4 => 2, 5 => 3, 6 => 0, 7 => 1
/// \param origin the node whose neighbor is needed
/// \param direction a set of directions, for face neighbors use 1, for edge neighbors use 2 and for vertex neighbors
///        use 3 direction, fill the other spots with SYMBOL_HALT. E.g. to find the left neighbor use
///        {SYMBOL_LEFT, SYMBOL_HALT, SYMBOL_HALT}, for the vertex neighber on the upper left front corner use
///        {SYMBOL_LEFT, SYMBOLD_FRONT, SYMBOL_UP}
/// \return a OctreePath to the correct neighbor, empty if the neighbor is outside of the tree
OctreePath getNeighbor(const OctreePath& origin, const std::array<Symbol, 3>& direction);

/// Fetch a list of neighbors, indicated by the type, Face, Edge and Vertex are possible types and can be combined
/// via OR
/// \param origin the node whose neighbor is needed
/// \param type the kind of neighborhood that is needed, \sa Neighborhood
/// \return list of paths with neighbors of the node at origin
std::vector<OctreePath> getNeighbors(const OctreePath& origin, int type);

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

	std::string getClassName() const override;

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
	/// \note The data stored in the current node will not be automatically subdivided.
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
	/// \throws SurgSim::Framework::AssertionFailure if the index >= 8
	/// \param index the child index
	/// \return the requested octree node
	std::shared_ptr<OctreeNode<Data> > getChild(size_t index);

	/// Get a child of this node
	/// \throws SurgSim::Framework::AssertionFailure if the index >= 8
	/// \param index the child index
	/// \return the requested octree node
	const std::shared_ptr<OctreeNode<Data> > getChild(size_t index) const;

	/// Get the node at the supplied path
	/// \throws SurgSim::Framework::AssertionFailure if returnLastValid is false and the node does not exist.
	/// \param path the path to the specific node
	/// \param returnLastValid if true and the path is longer than the tree deep, the function will return
	//                         the last node on a given path, otherwise it will throw.
	/// \return the requested octree node
	virtual std::shared_ptr<OctreeNode<Data> > getNode(const OctreePath& path, bool returnLastValid = false);

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

	bool doLoad(const std::string& filePath) override;

	/// The bounding box of the current OctreeNode
	SurgSim::Math::Aabbd m_boundingBox;

	/// True if there is any data inside this node, including data held by children, children's children, etc.
	bool m_isActive;

	/// True if the node has children
	bool m_hasChildren;

	/// The children of this node
	std::array<std::shared_ptr<OctreeNode<Data> >, 8> m_children;

private:
	static std::string m_className;
};


/// A free function to load an octree from file.
/// \param fileName	Name of the external file which contains an octree.
/// \return A std::shared_ptr<> pointing to an OctreeNode.
std::shared_ptr<OctreeNode<SurgSim::DataStructures::EmptyData>> loadOctree(const std::string& fileName);

};  // namespace DataStructures
};  // namespace SurgSim

#include "SurgSim/DataStructures/OctreeNode-inl.h"

#endif // SURGSIM_DATASTRUCTURES_OCTREENODE_H
