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

#include "SurgSim/DataStructures/OctreeNodePlyReaderDelegate.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace DataStructures
{

OctreeNodePlyReaderDelegateBase::OctreeNodePlyReaderDelegateBase() :
	m_haveSpacing(false),
	m_haveDimensions(false),
	m_haveBounds(false)
{

}

OctreeNodePlyReaderDelegateBase::~OctreeNodePlyReaderDelegateBase()
{

}

bool OctreeNodePlyReaderDelegateBase::registerDelegate(PlyReader* reader)
{
	// Vertex processing
	reader->requestElement("bounds",
						   std::bind(&OctreeNodePlyReaderDelegateBase::beginBounds, this,
									 std::placeholders::_1, std::placeholders::_2),
						   nullptr,
						   nullptr);
	reader->requestScalarProperty("bounds", "xMin", PlyReader::TYPE_DOUBLE, offsetof(BoundsData, xMin));
	reader->requestScalarProperty("bounds", "yMin", PlyReader::TYPE_DOUBLE, offsetof(BoundsData, yMin));
	reader->requestScalarProperty("bounds", "zMin", PlyReader::TYPE_DOUBLE, offsetof(BoundsData, zMin));
	reader->requestScalarProperty("bounds", "xMax", PlyReader::TYPE_DOUBLE, offsetof(BoundsData, xMax));
	reader->requestScalarProperty("bounds", "yMax", PlyReader::TYPE_DOUBLE, offsetof(BoundsData, yMax));
	reader->requestScalarProperty("bounds", "zMax", PlyReader::TYPE_DOUBLE, offsetof(BoundsData, zMax));

	reader->requestElement("dimension",
						   std::bind(&OctreeNodePlyReaderDelegateBase::beginDimension, this,
									 std::placeholders::_1, std::placeholders::_2),
						   nullptr,
						   nullptr);
	reader->requestScalarProperty("dimension", "x", PlyReader::TYPE_UNSIGNED_INT, offsetof(DimensionData, x));
	reader->requestScalarProperty("dimension", "y", PlyReader::TYPE_UNSIGNED_INT, offsetof(DimensionData, y));
	reader->requestScalarProperty("dimension", "z", PlyReader::TYPE_UNSIGNED_INT, offsetof(DimensionData, z));

	reader->requestElement("spacing",
						   std::bind(&OctreeNodePlyReaderDelegateBase::beginSpacing, this,
									 std::placeholders::_1, std::placeholders::_2),
						   nullptr,
						   nullptr);
	reader->requestScalarProperty("spacing", "x", PlyReader::TYPE_DOUBLE, offsetof(SpacingData, x));
	reader->requestScalarProperty("spacing", "y", PlyReader::TYPE_DOUBLE, offsetof(SpacingData, y));
	reader->requestScalarProperty("spacing", "z", PlyReader::TYPE_DOUBLE, offsetof(SpacingData, z));

	reader->requestElement("voxel",
						   std::bind(&OctreeNodePlyReaderDelegateBase::beginVoxel, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&OctreeNodePlyReaderDelegateBase::processVoxel, this, std::placeholders::_1),
						   nullptr);
	reader->requestScalarProperty("voxel", "x", PlyReader::TYPE_DOUBLE, offsetof(VoxelData, x));
	reader->requestScalarProperty("voxel", "y", PlyReader::TYPE_DOUBLE, offsetof(VoxelData, y));
	reader->requestScalarProperty("voxel", "z", PlyReader::TYPE_DOUBLE, offsetof(VoxelData, z));

	return true;
}

bool OctreeNodePlyReaderDelegateBase::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("bounds", "xMin") && reader.hasProperty("bounds", "yMin") &&
			 reader.hasProperty("bounds", "zMin") && reader.hasProperty("bounds", "xMax") &&
			 reader.hasProperty("bounds", "yMax") && reader.hasProperty("bounds", "zMax");
	result = result && reader.hasProperty("dimension", "x") && reader.hasProperty("dimension", "y") &&
			 reader.hasProperty("dimension", "z");
	result = result && reader.hasProperty("spacing", "x") && reader.hasProperty("spacing", "y") &&
			 reader.hasProperty("spacing", "z");
	result = result && reader.hasProperty("voxel", "x") && reader.hasProperty("voxel", "y") &&
			 reader.hasProperty("voxel", "z");

	return result;
}

void* OctreeNodePlyReaderDelegateBase::beginBounds(const std::string& elementName, size_t count)
{
	SURGSIM_ASSERT(count == 1) << "Bounds should only have exactly one entry.";
	m_haveBounds = true;
	return &m_bounds;
}

void* OctreeNodePlyReaderDelegateBase::beginDimension(const std::string& elementName, size_t count)
{
	SURGSIM_ASSERT(count == 1) << "Dimension should only have exactly one entry.";
	m_haveDimensions = true;
	return &m_dimension;
}

void* OctreeNodePlyReaderDelegateBase::beginSpacing(const std::string& elementName, size_t count)
{
	SURGSIM_ASSERT(count == 1) << "Spacing should only have exactly one entry.";
	m_haveSpacing = true;
	return &m_spacing;
}

void* OctreeNodePlyReaderDelegateBase::beginVoxel(const std::string& elementName, size_t count)
{
	SURGSIM_ASSERT(m_haveSpacing) << "Need spacing data for complete octree.";
	SURGSIM_ASSERT(m_haveBounds) << "Need bounds data for complete octree.";
	SURGSIM_ASSERT(m_haveDimensions) << "Need dimension data for complete octree.";

	auto maxDimension = std::max(m_dimension.x, std::max(m_dimension.y, m_dimension.z));

	m_numLevels = 1 + static_cast<int>(std::ceil(std::log(maxDimension) / std::log(2.0)));
	SurgSim::Math::Vector3d octreeDimensions = SurgSim::Math::Vector3d::Ones() * std::pow(2.0, m_numLevels - 1);

	Math::Vector3d spacing(m_spacing.x, m_spacing.y, m_spacing.z);

	m_boundingBox.min() = Math::Vector3d(m_bounds.xMin, m_bounds.yMin, m_bounds.zMin);
	m_boundingBox.max() = Math::Vector3d(m_bounds.xMin, m_bounds.yMin, m_bounds.zMin).array() +
						  octreeDimensions.array() * spacing.array();

	initializeOctree();

	return &m_voxel;
}

}
}

