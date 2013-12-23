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
//

#ifndef SURGSIM_MATH_OCTREESHAPE_INL_H
#define SURGSIM_MATH_OCTREESHAPE_INL_H

namespace SurgSim
{
namespace Math
{

template<class Data>
std::shared_ptr<OctreeShape> OctreeShape::fromOctreeNode(
		std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> node)
{
	std::shared_ptr<OctreeShape::NodeType> myNode = std::make_shared<OctreeShape::NodeType>();
	SurgSim::DataStructures::copyOctreeNode(node, myNode);
	std::shared_ptr<OctreeShape> shape = std::make_shared<OctreeShape>();
	shape->setRootNode(myNode);
	return shape;
}

}; // namespace Math
}; // namespace SurgSim

#endif // SURGSIM_MATH_OCTREESHAPE_INL_H
