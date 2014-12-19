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

#include <array>
#include <cmath>
#include <fstream>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/DataStructures/OctreeNodePlyReaderDelegate.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{

namespace DataStructures
{

template<class Data>
OctreeNode<Data>::OctreeNode() :
	m_isActive(false),
	m_hasChildren(false)
{
}


template<class Data>
OctreeNode<Data>::OctreeNode(const SurgSim::Math::Aabbd& boundingBox) :
	m_boundingBox(boundingBox),
	m_isActive(false),
	m_hasChildren(false)
{
}

template<class Data>
SurgSim::DataStructures::OctreeNode<Data>::OctreeNode(const OctreeNode& other)
{
	m_boundingBox = other.m_boundingBox;
	m_hasChildren = other.m_hasChildren;
	m_isActive = other.m_isActive;

	// Also copy the data since they are the same type
	data = other.data;

	for (size_t i = 0; i < other.m_children.size(); i++)
	{
		if (other.getChild(i) == nullptr)
		{
			m_children[i] = nullptr;
		}
		else
		{
			m_children[i] = std::make_shared<OctreeNode<Data>>(*other.m_children[i]);
		}
	}
}

template <class Data>
template <class T>
SurgSim::DataStructures::OctreeNode<Data>::OctreeNode(const OctreeNode<T>& other)
{
	m_boundingBox = other.getBoundingBox();
	m_hasChildren = other.hasChildren();
	m_isActive = other.isActive();

	for (size_t i = 0; i < m_children.size(); i++)
	{
		auto child = other.getChild(i);
		if (child == nullptr)
		{
			m_children[i] = nullptr;
		}
		else
		{
			m_children[i] = std::make_shared<OctreeNode<Data>>(*child);
		}
	}
}

template<class Data>
OctreeNode<Data>::~OctreeNode()
{
}

template<class Data>
std::string OctreeNode<Data>::getClassName() const
{
	return m_className;
}

template<class Data>
const SurgSim::Math::Aabbd& OctreeNode<Data>::getBoundingBox() const
{
	return m_boundingBox;
}

template<class Data>
bool OctreeNode<Data>::isActive() const
{
	return m_isActive;
}

template<class Data>
void OctreeNode<Data>::setIsActive(bool isActive)
{
	m_isActive = isActive;
}

template<class Data>
bool OctreeNode<Data>::hasChildren() const
{
	return m_hasChildren;
}

template<class Data>
void OctreeNode<Data>::subdivide()
{
	using SurgSim::Math::Vector3d;

	if (! m_hasChildren)
	{
		Vector3d childsSize = (m_boundingBox.max() - m_boundingBox.min()) / 2.0;
		AxisAlignedBoundingBox childsBoundingBox;
		for (int i = 0; i < 8; i++)
		{
			// Use the index to pick one of the eight regions
			Vector3d regionIndex = Vector3d(((i & 1) == 0) ? 0 : 1,
											((i & 2) == 0) ? 0 : 1,
											((i & 4) == 0) ? 0 : 1);
			childsBoundingBox.min() = m_boundingBox.min().array() + regionIndex.array() * childsSize.array();
			childsBoundingBox.max() = childsBoundingBox.min() + childsSize;
			m_children[i] = std::make_shared<OctreeNode<Data>>(childsBoundingBox);
		}
		m_hasChildren = true;
	}
}

template<class Data>
bool OctreeNode<Data>::addData(const SurgSim::Math::Vector3d& position, const Data& nodeData, const int level)
{
	return doAddData(position, nodeData, level, 1);
}


template<class Data>
bool OctreeNode<Data>::addDefaultData(const SurgSim::Math::Vector3d& position, int level)
{
	return doAddData(position, Data(), level, 1);
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

	if (! m_hasChildren)
	{
		subdivide();
	}
	for (auto child = m_children.begin(); child != m_children.end(); ++child)
	{
		if ((*child)->doAddData(position, nodeData, level, currentLevel + 1))
		{
			m_isActive = true;
			return true;
		}
	}
	return false;
}

template<class Data>
std::array<std::shared_ptr<OctreeNode<Data>>, 8>& OctreeNode<Data>::getChildren()
{
	return m_children;
}

template<class Data>
const std::array<std::shared_ptr<OctreeNode<Data>>, 8>& OctreeNode<Data>::getChildren() const
{
	return m_children;
}

template<class Data>
std::shared_ptr<OctreeNode<Data>> OctreeNode<Data>::getChild(size_t index)
{
	return m_children[index];
}

template<class Data>
const std::shared_ptr<OctreeNode<Data>> OctreeNode<Data>::getChild(size_t index) const
{
	return m_children[index];
}

template<class Data>
std::shared_ptr<OctreeNode<Data>> OctreeNode<Data>::getNode(const OctreePath& path, bool returnLastValid)
{
	std::shared_ptr<OctreeNode<Data>> node = this->shared_from_this();
	std::shared_ptr<OctreeNode<Data>> previous;
	for (auto index = path.cbegin(); index != path.cend(); ++index)
	{
		previous = std::move(node);
		node = previous->getChild(*index);
		if (node == nullptr)
		{
			if (returnLastValid)
			{
				node = std::move(previous);
				break;
			}
			else
			{
				SURGSIM_FAILURE() << "Octree path is invalid. Path is longer than octree is deep in this given branch.";
			}
		}
	}
	return node;
}

template<class Data>
bool SurgSim::DataStructures::OctreeNode<Data>::doLoad(const std::string& fileName)
{
	SurgSim::Framework::Timer timer;
	auto delegate = std::make_shared<OctreeNodePlyReaderDelegate<Data>>(this->shared_from_this());

	PlyReader reader(fileName);
	SURGSIM_ASSERT(reader.isValid()) << "'" << fileName << "' is an invalid .ply file.";
	SURGSIM_ASSERT(reader.parseWithDelegate(delegate)) <<
			"The input file " << fileName << " does not have the property required by the octree.";
	timer.endFrame();
	SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getDefaultLogger())
			<< "Loading " << fileName << " took " << timer.getCumulativeTime() << "s. ";
	return true;
}

};  // namespace DataStructures

};  // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_OCTREENODE_INL_H
