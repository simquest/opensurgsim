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

/// \file
/// Unit Tests for the OsgOctreeRepresentation class.

#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Graphics/OsgOctreeRepresentation.h"

#include <gtest/gtest.h>

using SurgSim::DataStructures::OctreeNode;
using SurgSim::Graphics::OsgOctreeRepresentation;
using SurgSim::Math::Vector3d;

struct EmptyData{}emptyData;

TEST(OsgOctreeRepresentationTests, GetSetTest)
{

	Eigen::AlignedBox<double, 3> boundingBox;
	boundingBox.min() = Vector3d::Zero();
	boundingBox.max() = Vector3d::Ones() * pow(2.0, 2);
	auto octree = std::make_shared<OctreeNode<EmptyData>>(boundingBox);

	auto octreeRepresentation =	std::make_shared<OsgOctreeRepresentation<EmptyData>>("Octree");

	octreeRepresentation->setOctree(octree);
	EXPECT_TRUE(octree == octreeRepresentation->getOctree());
}