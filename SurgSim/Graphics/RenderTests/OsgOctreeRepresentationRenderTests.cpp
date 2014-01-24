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

/// Render Tests for the OsgOctreeRepresentation class.

#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Graphics/OctreeRepresentation.h"
#include "SurgSim/Graphics/OsgOctreeRepresentation.h"
#include "SurgSim/Math/OctreeShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Graphics::OsgOctreeRepresentation;
using SurgSim::Graphics::OctreeRepresentation;
using SurgSim::Math::OctreeShape;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

struct OsgOctreeRepresentationRenderTests : public SurgSim::Graphics::RenderTest
{
};


// An Octree(Node) is traversed in following order (the 2nd OctreeNode, i.e. OctreeNode with "1" is now shown):
/*
				________
			   /3  /  7/|
			  /-------/ |
			 /2__/_6_/| |
			|   |   | |/|
			|___|___|/|5|
			|   |   | |/
			|0__|__4|/
*/
TEST_F(OsgOctreeRepresentationRenderTests, OctreeSubdivide)
{
	SurgSim::Math::OctreeShape::EmptyData emptyData;

	SurgSim::Math::OctreeShape::NodeType::AxisAlignedBoundingBox boundingBox;
	boundingBox.min() = Vector3d::Ones() * -2.0;
	boundingBox.max() = Vector3d::Ones() * 2.0;

	auto octree1 = std::make_shared<OctreeShape::NodeType>(boundingBox);
	auto octreeShape = std::make_shared<OctreeShape>();
	octreeShape->setRootNode(octree1);

	auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation>("Octree Representation");
	octreeRepresentation->setOctree(octreeShape);

	octreeRepresentation->setInitialPose(makeRigidTransform(
										  makeRotationQuaternion(M_PI_4, Vector3d(1.0, 1.0, 1.0)),
										  Vector3d(0.0, 0.0, -20.0))
										);
	viewElement->addComponent(octreeRepresentation);

	auto octree2 = octreeRepresentation->getOctree();
	// An OsgOctreeRepresentation holds a copy of the Octree.
	EXPECT_NE(octree2, octree1);

	std::array<Vector3d, 8> secondLevelPositions = {{
			Vector3d(-1.0, -1.0, -1.0),
			Vector3d( 1.0, -1.0, -1.0),
			Vector3d(-1.0,  1.0, -1.0),
			Vector3d( 1.0,  1.0, -1.0),
			Vector3d(-1.0, -1.0,  1.0),
			Vector3d( 1.0, -1.0,  1.0),
			Vector3d(-1.0,  1.0,  1.0),
			Vector3d( 1.0,  1.0,  1.0)}};
	octree2->addData(secondLevelPositions[0], emptyData, 2);
	octree2->addData(secondLevelPositions[1], emptyData, 2);
	octree2->addData(secondLevelPositions[3], emptyData, 2);
	octree2->addData(secondLevelPositions[7], emptyData, 2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
}
