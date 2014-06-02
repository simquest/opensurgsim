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
/// Tests for the OsgPlane class.

#include "SurgSim/Graphics/OsgPlane.h"

#include <gtest/gtest.h>

namespace SurgSim
{

namespace Graphics
{

TEST(OsgPlaneTests, InitTest)
{
	ASSERT_NO_THROW({OsgPlane plane;});

	ASSERT_NO_THROW({OsgPlane plane(100.0, 200.0);});

	OsgPlane plane;

	osg::ref_ptr<osg::Node> node = plane.getNode();
	EXPECT_NE(nullptr, node.get());

	osg::ref_ptr<osg::Geode> geode = dynamic_cast<osg::Geode*>(node.get());
	EXPECT_NE(nullptr, geode.get());

	EXPECT_EQ(1u, geode->getNumDrawables());

	osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(0);
	EXPECT_NE(nullptr, drawable.get());

	osg::ref_ptr<osg::Geometry> geometry = dynamic_cast<osg::Geometry*>(drawable.get());
	EXPECT_NE(nullptr, geometry.get());

	osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
	ASSERT_EQ(4u, vertices->size());

	osg::Vec3 u = vertices->at(1) - vertices->at(0);
	osg::Vec3 v = vertices->at(2) - vertices->at(0);
	osg::Vec3 normal = u ^ v;
	normal.normalize();

	EXPECT_EQ(0.0, vertices->at(0).y()) << "The plane should be in the XZ plane (Y = 0)!";

	EXPECT_NEAR(1.0, normal * osg::Vec3(0.0, 1.0, 0.0), 1.0e-10) << "The plane normal should be +Y!";

	EXPECT_NEAR(0.0, vertices->at(3) * normal, 1.0e-10) << "All vertices should be co-planar!";

	osg::Vec3 sum(0.0, 0.0, 0.0);
	for (size_t i = 0; i < vertices->size(); ++i)
	{
		sum += vertices->at(i);
	}
	sum /= vertices->size();

	EXPECT_NEAR(0.0, sum.length(), 1e-10) << "The center of the plane should be (0, 0, 0)!";
}

};  // namespace Graphics

};  // namespace SurgSim
