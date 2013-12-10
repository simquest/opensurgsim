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

#ifndef SURGSIM_MATH_OCTREESHAPE_INL_H
#define SURGSIM_MATH_OCTREESHAPE_INL_H

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{

namespace Math
{

template<class Data>
OctreeShape<Data>::OctreeShape(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> node) :
	m_rootNode(node)
{
}

template<class Data>
OctreeShape<Data>::~OctreeShape()
{
}

template<class Data>
int OctreeShape<Data>::getType()
{
	return SHAPE_TYPE_OCTREE;
}

template<class Data>
double OctreeShape<Data>::calculateVolume() const
{
	SURGSIM_FAILURE() << "OctreeShape::calculateVolume not implemented";
	return 0.0;
}

template<class Data>
double OctreeShape<Data>::calculateMass(double rho) const
{
	SURGSIM_FAILURE() << "OctreeShape::calculateMass not implemented";
	return 0.0;
}

template<class Data>
Vector3d OctreeShape<Data>::calculateMassCenter() const
{
	return Vector3d::Zero();
}

template<class Data>
Matrix33d OctreeShape<Data>::calculateInertia(double rho) const
{
	SURGSIM_FAILURE() << "OctreeShape::calculateInertia not implemented";
	return Matrix33d::Zero();
}

template<class Data>
std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> OctreeShape<Data>::getRootNode()
{
	return m_rootNode;
}

template<class Data>
void OctreeShape<Data>::setRootNode(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> node)
{
	m_rootNode = node;
}

template<class Data>
std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> OctreeShape<Data>::getNode(const OctreePath& path)
{
	std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> node = m_rootNode;
	for (auto index = path.cbegin(); index != path.cend(); ++index)
	{
		node = node->getChild(*index);
		SURGSIM_ASSERT(node != nullptr)
			<< "Octree path is invalid. Path is longer than octree is deep in this given branch.";
	}
	return node;
}

template<class Data>
std::string OctreeShape<Data>::getClassName()
{
	return std::string("SurgSim::Math::OctreeShape");
}

}; // namespace Math

}; // namespace SurgSim



#endif // SURGSIM_MATH_OCTREESHAPE_INL_H
