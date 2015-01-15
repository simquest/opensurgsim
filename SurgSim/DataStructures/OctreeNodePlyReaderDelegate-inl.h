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

#ifndef SURGSIM_DATASTRUCTURES_OCTREENODEPLYREADERDELEGATE_INL_H
#define SURGSIM_DATASTRUCTURES_OCTREENODEPLYREADERDELEGATE_INL_H

namespace SurgSim
{
namespace DataStructures
{

template <typename Data>
OctreeNodePlyReaderDelegate<Data>::OctreeNodePlyReaderDelegate() :
	m_octree(std::make_shared<OctreeNode<Data>>())
{
	static_assert(std::is_default_constructible<Data>::value, "OctreeNode Data needs default constructor.");
}

template <typename Data>
OctreeNodePlyReaderDelegate<Data>::OctreeNodePlyReaderDelegate(std::shared_ptr<OctreeNode<Data>> octree) :
	m_octree(octree)
{
	static_assert(std::is_default_constructible<Data>::value, "OctreeNode Data needs default constructor");
	SURGSIM_ASSERT(!m_octree->hasChildren()) << "Can't process an octree that already has children in it.";
}

template <typename Data>
OctreeNodePlyReaderDelegate<Data>::~OctreeNodePlyReaderDelegate()
{

}

template <typename Data>
std::shared_ptr<OctreeNode<Data>> OctreeNodePlyReaderDelegate<Data>::getOctree()
{
	return m_octree;
}

template <typename Data>
void OctreeNodePlyReaderDelegate<Data>::processVoxel(const std::string& elementName)
{
	SurgSim::Math::Vector3d position;
	position[0] = m_voxel.x;
	position[1] = m_voxel.y;
	position[2] = m_voxel.z;
	m_octree->addData(position, m_numLevels);
}

template <typename Data>
void OctreeNodePlyReaderDelegate<Data>::initializeOctree()
{
	m_octree->m_boundingBox = m_boundingBox;
}

}
}

#endif
