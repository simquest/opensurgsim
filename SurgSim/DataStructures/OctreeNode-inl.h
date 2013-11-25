// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_OCTREENODE_INL_H
#define SURGSIM_DATASTRUCTURES_OCTREENODE_INL_H

namespace SurgSim
{

namespace DataStructures
{


template<class Data>
OctreeNode<Data>::OctreeNode(const OctreeNode<Data>::BoundingBoxType& boundingBox) :
	m_boundingBox(boundingBox),
	m_isActive(false),
	m_isLeafNode(true),
	m_children(0)
{
}

template<class Data>
OctreeNode<Data>::~OctreeNode()
{
}

template<class Data>
const typename OctreeNode<Data>::BoundingBoxType& OctreeNode<Data>::getBoundingBox() const
{
	return m_boundingBox;
}

template<class Data>
bool OctreeNode<Data>::isActive() const
{
	return m_isActive;
}

template<class Data>
bool OctreeNode<Data>::isLeafNode() const
{
	return m_isLeafNode;
}

template<class Data>
void OctreeNode<Data>::subdivide()
{
	using SurgSim::Math::Vector3d;

	if (m_isLeafNode)
	{
		m_children.resize(8);

		Vector3d childsSize = (m_boundingBox.max() - m_boundingBox.min()) / 2.0;
		BoundingBoxType childsBoundingBox;
		for (int i=0; i<8; i++)
		{
			// Use the index to pick one of the eight regions
			Vector3d regionIndex = Vector3d(((i & 1) == 0) ? 0 : 1,
											((i & 2) == 0) ? 0 : 1,
											((i & 4) == 0) ? 0 : 1);
			childsBoundingBox.min() = m_boundingBox.min().array() + regionIndex.array() * childsSize.array();
			childsBoundingBox.max() = childsBoundingBox.min() + childsSize;
			m_children[i] = std::make_shared<OctreeNode<Data>>(childsBoundingBox);
		}
		m_isLeafNode = false;
	}
}

template<class Data>
bool OctreeNode<Data>::addData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level)
{
	return doAddData(position, nodeData, level, 1);
}

template<class Data>
bool OctreeNode<Data>::doAddData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level,
		const int currentLevel)
{
	if (! m_boundingBox.contains(position))
	{
		return false;
	}

	if (currentLevel == level)
	{
		data = nodeData;
		m_isActive = true;
		return true;
	}

	if (m_isLeafNode)
	{
		subdivide();
	}
	for (auto child=m_children.begin(); child!=m_children.end(); ++child)
	{
		if ((*child)->doAddData(position, nodeData, level, currentLevel+1))
		{
			m_isActive = true;
			return true;
		}
	}
	return false;
}

template<class Data>
std::vector<std::shared_ptr<OctreeNode<Data> > >& OctreeNode<Data>::getChildren()
{
	return m_children;
}

template<class Data>
const std::vector<std::shared_ptr<OctreeNode<Data> > >& OctreeNode<Data>::getChildren() const
{
	return m_children;
}


};  // namespace DataStructures

};  // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_OCTREENODE_INL_H
