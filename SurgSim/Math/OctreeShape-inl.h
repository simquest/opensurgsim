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

namespace SurgSim
{

namespace Math
{

template<class NodeData>
OctreeShape<NodeData>::OctreeShape(std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> node) :
	m_rootNode(node)
{
}

template<class NodeData>
OctreeShape<NodeData>::~OctreeShape()
{
}

template<class NodeData>
int OctreeShape<NodeData>::getType()
{
	return SHAPE_TYPE_OCTREE;
}

template<class NodeData>
double OctreeShape<NodeData>::calculateVolume() const
{
	return 0.0;
}

template<class NodeData>
double OctreeShape<NodeData>::calculateMass(double rho) const
{
	return 0.0;
}

template<class NodeData>
Vector3d OctreeShape<NodeData>::calculateMassCenter() const
{
	return Vector3d::Zero();
}

template<class NodeData>
Matrix33d OctreeShape<NodeData>::calculateInertia(double rho) const
{
	return Matrix33d::Zero();
}

template<class NodeData>
std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> OctreeShape<NodeData>::getRootNode()
{
	return m_rootNode;
}

template<class NodeData>
void OctreeShape<NodeData>::setRootNode(std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> node)
{
	m_rootNode = node;
}

template<class NodeData>
std::string OctreeShape<NodeData>::getClassName()
{
	return std::string("SurgSim::Math::OctreeShape");
}

}; // namespace Math

}; // namespace SurgSim



#endif // SURGSIM_MATH_OCTREESHAPE_INL_H
