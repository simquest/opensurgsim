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


#include "SurgSim/Math/OctreeShape.h"

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{

namespace Math
{

OctreeShape::OctreeShape()
{
}

OctreeShape::~OctreeShape()
{
}

int OctreeShape::getType()
{
	return SHAPE_TYPE_OCTREE;
}

double OctreeShape::getVolume() const
{
	SURGSIM_FAILURE() << "OctreeShape::getVolume not implemented";
	return 0.0;
}

Vector3d OctreeShape::getCenter() const
{
	return Vector3d::Zero();
}

Matrix33d OctreeShape::getSecondMomentOfVolume() const
{
	SURGSIM_FAILURE() << "OctreeShape::getSecondMomentOfVolume not implemented";
	return Matrix33d::Zero();
}

std::shared_ptr<OctreeShape::NodeType> OctreeShape::getRootNode() const
{
	return m_rootNode;
}

void OctreeShape::setRootNode(std::shared_ptr<OctreeShape::NodeType> node)
{
	m_rootNode = node;
}

std::string OctreeShape::getClassName()
{
	return std::string("SurgSim::Math::OctreeShape");
}

}; // namespace Math

}; // namespace SurgSim

