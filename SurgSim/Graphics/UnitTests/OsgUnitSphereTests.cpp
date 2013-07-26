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
/// Tests for the OsgUnitSphere class.

#include <SurgSim/Graphics/OsgUnitSphere.h>

#include <gtest/gtest.h>

namespace SurgSim
{

namespace Graphics
{

TEST(OsgUnitSphereTests, InitTest)
{
	ASSERT_NO_THROW({OsgUnitSphere sphere;});

	OsgUnitSphere sphere;

	osg::ref_ptr<osg::Node> node = sphere.getNode();
	EXPECT_NE(nullptr, node.get());

	osg::ref_ptr<osg::PositionAttitudeTransform> transform = dynamic_cast<osg::PositionAttitudeTransform*>(node.get());
	EXPECT_NE(nullptr, transform.get());

	osg::Quat rotation;
	rotation.makeRotate(osg::Vec3d(0.0, 0.0, 1.0), osg::Vec3d(0.0, 1.0, 0.0));
	EXPECT_EQ(rotation, transform->getAttitude());
	EXPECT_EQ(osg::Vec3(0.0, 0.0, 0.0), transform->getPosition());
	EXPECT_EQ(osg::Vec3(1.0, 1.0, 1.0), transform->getScale());

	EXPECT_EQ(1u, transform->getNumChildren());

	osg::ref_ptr<osg::Geode> geode = dynamic_cast<osg::Geode*>(transform->getChild(0u));
	EXPECT_NE(nullptr, geode.get());

	EXPECT_EQ(1u, geode->getNumDrawables());

	osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(0);
	EXPECT_NE(nullptr, drawable.get());

	osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = dynamic_cast<osg::ShapeDrawable*>(drawable.get());
	EXPECT_NE(nullptr, shapeDrawable.get());

	osg::ref_ptr<osg::Shape> shape = shapeDrawable->getShape();
	EXPECT_NE(nullptr, shape.get());

	osg::ref_ptr<osg::Sphere> sphereShape = dynamic_cast<osg::Sphere*>(shape.get());
	EXPECT_NE(nullptr, sphereShape.get());

	/// The sphere should be at (0, 0, 0) with radius 1
	EXPECT_EQ(0.0f, sphereShape->getCenter().x());
	EXPECT_EQ(0.0f, sphereShape->getCenter().y());
	EXPECT_EQ(0.0f, sphereShape->getCenter().z());
	EXPECT_EQ(1.0f, sphereShape->getRadius());
}

};  // namespace Graphics

};  // namespace SurgSim
