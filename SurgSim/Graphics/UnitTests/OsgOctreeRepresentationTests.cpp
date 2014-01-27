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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgOctreeRepresentation.h"
#include "SurgSim/Math/OctreeShape.h"

#include <gtest/gtest.h>

using SurgSim::Framework::Runtime;
using SurgSim::Graphics::OsgOctreeRepresentation;
using SurgSim::Math::OctreeShape;
using SurgSim::Math::Vector3d;

TEST(OsgOctreeRepresentationTests, InitilizationTest)
{
	ASSERT_NO_THROW( {auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation>("Test Octree");} );
};

TEST(OsgOctreeRepresentationTests, GetSetUpdateTest)
{
	OctreeShape::NodeType::AxisAlignedBoundingBox boundingBox(Vector3d::Zero(), Vector3d::Ones() * pow(2.0, 2));
	auto octreeNode = std::make_shared<OctreeShape::NodeType>(boundingBox);
	OctreeShape octreeShape;
	octreeShape.setRootNode(octreeNode);

	auto runtime = std::make_shared<Runtime>();
	auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation>("Test Octree");
	octreeRepresentation->setOctree(octreeShape);

	// OsgOctreeRepresentation owns a local copy of Octree, not shared with OctreeShape.
	EXPECT_FALSE(octreeNode == octreeRepresentation->getOctree());

	EXPECT_TRUE(octreeRepresentation->initialize(runtime));
	EXPECT_TRUE(octreeRepresentation->wakeUp());

	// Set the octree after wake up will cause a assertion failure.
	ASSERT_ANY_THROW( { octreeRepresentation->setOctree(octreeShape); } );

	ASSERT_NO_THROW(octreeRepresentation->update(0.1));
}