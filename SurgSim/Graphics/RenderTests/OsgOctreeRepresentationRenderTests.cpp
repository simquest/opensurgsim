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
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Testing/MathUtilities.h"

using SurgSim::Graphics::OsgOctreeRepresentation;
using SurgSim::Graphics::OctreeRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Testing::interpolate;

struct EmptyData{}emptyData;

struct OsgOctreeRepresentationRenderTests : public SurgSim::Graphics::RenderTest
{
};

TEST_F(OsgOctreeRepresentationRenderTests, OctreeSubdivide)
{
	auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation<EmptyData>>("Octree Representation");

	octreeRepresentation->setInitialPose(makeRigidTransform(
										 makeRotationQuaternion(M_PI_4, Vector3d(1.0, 1.0, 1.0)),
										 Vector3d(0.0, 0.0, -8.0)));
	viewElement->addComponent(octreeRepresentation);

	octreeRepresentation->getOctree()->addData(Vector3d(0.0, 0.0, 0.0), emptyData, 1);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	octreeRepresentation->getOctree()->subdivide();
	for (int i = 0; i < 8; ++i)
	{
		auto boundingBox = octreeRepresentation->getOctree()->getChild(i)->getBoundingBox();
		Vector3d center = (boundingBox.max() + boundingBox.min()) / 2.0;
		octreeRepresentation->getOctree()->addData(center, emptyData, 2);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}

	for (int i = 0; i < 8; ++i)
	{
		octreeRepresentation->getOctree()->getChild(i)->subdivide();
	}

	for (int i = 0; i < 8; ++i)
	{
		auto self = octreeRepresentation->getOctree()->getChild(i);
		for (int j = 0; j < 8; ++j)
		{
			auto childBox = self->getChild(j)->getBoundingBox();
			Vector3d childCenter = (childBox.max() + childBox.min()) / 2.0;

			if ( childCenter.cwiseAbs().sum()  <= 0.75)
			{
				octreeRepresentation->getOctree()->addData(childCenter, emptyData, 3);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			}
		}
	}
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
}
