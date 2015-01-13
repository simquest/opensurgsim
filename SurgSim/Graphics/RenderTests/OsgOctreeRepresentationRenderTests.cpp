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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Graphics/OctreeRepresentation.h"
#include "SurgSim/Graphics/OsgOctreeRepresentation.h"
#include "SurgSim/Math/OctreeShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Graphics::OsgOctreeRepresentation;
using SurgSim::Graphics::OctreeRepresentation;
using SurgSim::Math::OctreeShape;
using SurgSim::Math::Vector3d;
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
	SurgSim::DataStructures::EmptyData emptyData;

	SurgSim::Math::OctreeShape::NodeType::AxisAlignedBoundingBox boundingBox;
	boundingBox.min() = Vector3d::Ones() * -0.2;
	boundingBox.max() = Vector3d::Ones() * 0.2;

	std::array<Vector3d, 8> secondLevelPositions = {{
			Vector3d(-0.1, -0.1, -0.1),
			Vector3d(0.1, -0.1, -0.1),
			Vector3d(-0.1,  0.1, -0.1),
			Vector3d(0.1,  0.1, -0.1),
			Vector3d(-0.1, -0.1,  0.1),
			Vector3d(0.1, -0.1,  0.1),
			Vector3d(-0.1,  0.1,  0.1),
			Vector3d(0.1,  0.1,  0.1)
		}
	};
	auto octree = std::make_shared<OctreeShape::NodeType>(boundingBox);
	octree->addData(secondLevelPositions[0], 2, emptyData);
	octree->addData(secondLevelPositions[1], 2, emptyData);
	octree->addData(secondLevelPositions[3], 2, emptyData);
	octree->addData(secondLevelPositions[7], 2, emptyData);
	octree->addData(Vector3d(0.2, 0.2, 0.01), 3, emptyData);
	octree->addData(Vector3d(-0.2, -0.2, 0.2), 3, emptyData);
	octree->addData(Vector3d(0.01, 0.01, 0.11), 3, emptyData);
	octree->addData(Vector3d(0.01, 0.01, 0.11), 4, emptyData);

	auto octreeShape = std::make_shared<SurgSim::Math::OctreeShape>(*octree);

	auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation>("Octree Representation");
	octreeRepresentation->setLocalPose(makeRigidTransform(
										   makeRotationQuaternion(M_PI_4, Vector3d(1.0, 1.0, 1.0)),
										   Vector3d(0.0, 0.0, -1.0))
									  );
	viewElement->addComponent(octreeRepresentation);
	octreeRepresentation->setOctreeShape(octreeShape);

	// Path to a leaf node
	SurgSim::DataStructures::OctreePath path0;
	path0.push_back(2);

	// Path to a leaf node
	SurgSim::DataStructures::OctreePath path1;
	path1.push_back(7);
	path1.push_back(3);

	// Path to a internal OctreeNode
	SurgSim::DataStructures::OctreePath path2;
	path2.push_back(4);

	SurgSim::DataStructures::OctreePath invalidPath;
	invalidPath.push_back(0);
	invalidPath.push_back(1);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	octreeRepresentation->setNodeVisible(path0, true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	octreeRepresentation->setNodeVisible(path1, false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	octreeRepresentation->setNodeVisible(path2, false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	EXPECT_ANY_THROW(octreeRepresentation->setNodeVisible(invalidPath, true));
}
