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

#ifndef SURGSIM_DATASTRUCTURES_OCTREENODEPLYREADERDELEGATE_H
#define SURGSIM_DATASTRUCTURES_OCTREENODEPLYREADERDELEGATE_H

#include "SurgSim/DataStructures/PlyReaderDelegate.h"
#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/DataStructures/EmptyData.h"

namespace SurgSim
{
namespace DataStructures
{

/// Delegate to read Octrees from ply files, this is the abstract base class, to let us modify
/// loading by datatype
class OctreeNodePlyReaderDelegateBase : public PlyReaderDelegate
{
public:

	/// Constructor
	OctreeNodePlyReaderDelegateBase();

	/// Destructor
	virtual ~OctreeNodePlyReaderDelegateBase();

	bool registerDelegate(PlyReader* reader) override;

	bool fileIsAcceptable(const PlyReader& reader) override;

	/// Callback function, begin the processing of the bounds.
	/// \param elementName Name of the element.
	/// \param count Number of entries.
	/// \throws SurgSim::Framework::AssertionFailure if count != 1, only one bounds entry is expected
	/// \return memory for bounds data to the reader.
	void* beginBounds(const std::string& elementName, size_t count);

	/// Callback function, begin the processing of the dimension.
	/// \param elementName Name of the element.
	/// \param count Number of entries.
	/// \throws SurgSim::Framework::AssertionFailure if count != 1, only one dimesion entry is expected
	/// \return memory for dimension data to the reader.
	void* beginDimension(const std::string& elementName, size_t count);

	/// Callback function, begin the processing of the bounds.
	/// \param elementName Name of the element.
	/// \param count Number of entries.
	/// \throws SurgSim::Framework::AssertionFailure if count != 1, only one spacing entry is expected
	/// \return memory for spacing data to the reader.
	void* beginSpacing(const std::string& elementName, size_t count);

	/// Callback function, begin the processing of the voxels.
	/// \param elementName Name of the element.
	/// \param count Number of bounds entry.
	/// \return memory for voxel data to the reader.
	virtual void* beginVoxel(const std::string& elementName, size_t count);

	/// Callback function to process one voxel. This is left up to the subclasses, they might have to deal with
	/// data specific processing
	/// \param elementName Name of the element.
	virtual void processVoxel(const std::string& elementName) = 0;

protected:
	/// Set up the octree, this is left up to the derived classes
	virtual void initializeOctree() = 0;

	/// Data Structure to receive the bounds information
	struct BoundsData
	{
		double xMin;
		double yMin;
		double zMin;
		double xMax;
		double yMax;
		double zMax;
	} m_bounds;

	/// Bounding box, will be initialized from the file
	Eigen::AlignedBox<double, 3> m_boundingBox;

	/// Data structure to receive the spacing information from the file
	struct SpacingData
	{
		double x;
		double y;
		double z;
	} m_spacing;

	/// Data structure to receive the dimension information from the file
	struct DimensionData
	{
		unsigned int x;
		unsigned int y;
		unsigned int z;
	} m_dimension;

	/// Data structure to receive the specific voxel information from the file
	struct VoxelData
	{
		double x;
		double y;
		double z;
	} m_voxel;

	/// Calculated number of levels for the octree
	int m_numLevels;

	///@{
	/// Check wether we actually received the appropriate information, if not we can't initialize the octree
	bool m_haveSpacing;
	bool m_haveDimensions;
	bool m_haveBounds;
	///@}

};

/// Subclass the OctreeNodePLyReaderDelegateBase class to enable processing of the templated data, this should be
/// specialized to enable specific processing of various extended node types
/// \tparam Data The data the should be inside the Octree, needs to be default constructable
template <typename Data>
class OctreeNodePlyReaderDelegate : public OctreeNodePlyReaderDelegateBase
{
public:

	/// Constructor
	OctreeNodePlyReaderDelegate();

	/// Constructor
	/// \param octree read the data into this octree
	explicit OctreeNodePlyReaderDelegate(std::shared_ptr<OctreeNode<Data>> octree);

	/// Destructor
	virtual ~OctreeNodePlyReaderDelegate();

	/// \return the octree
	std::shared_ptr<OctreeNode<Data>> getOctree();

	void processVoxel(const std::string& elementName) override;

	void initializeOctree() override;

private:

	/// The octree that will be filled with the data from the file
	std::shared_ptr<OctreeNode<Data>> m_octree;
};

}
}

#include "SurgSim/DataStructures/OctreeNodePlyReaderDelegate-inl.h"

#endif
