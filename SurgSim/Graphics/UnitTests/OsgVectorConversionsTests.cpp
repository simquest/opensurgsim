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

/// \file
/// Tests for conversions to and from OSG vector types

#include "SurgSim/Graphics/OsgVectorConversions.h"

#include <gtest/gtest.h>

using SurgSim::Graphics::fromOsg;
using SurgSim::Graphics::toOsg;
using SurgSim::Math::Vector2f;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3f;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector4d;

TEST(OsgVectorConversionsTests, Vector2fTest)
{
	Vector2f vector = Vector2f::Random();
	osg::Vec2f osgVector = toOsg(vector);
	EXPECT_TRUE(vector.isApprox(fromOsg(osgVector)));
}

TEST(OsgVectorConversionsTests, Vector2dTest)
{
	Vector2d vector = Vector2d::Random();
	osg::Vec2d osgVector = toOsg(vector);
	EXPECT_TRUE(vector.isApprox(fromOsg(osgVector)));
}

TEST(OsgVectorConversionsTests, Vector3fTest)
{
	Vector3f vector = Vector3f::Random();
	osg::Vec3f osgVector = toOsg(vector);
	EXPECT_TRUE(vector.isApprox(fromOsg(osgVector)));
}

TEST(OsgVectorConversionsTests, Vector3dTest)
{
	Vector3d vector = Vector3d::Random();
	osg::Vec3d osgVector = toOsg(vector);
	EXPECT_TRUE(vector.isApprox(fromOsg(osgVector)));
}

TEST(OsgVectorConversionsTests, Vector4fTest)
{
	Vector4f vector = Vector4f::Random();
	osg::Vec4f osgVector = toOsg(vector);
	EXPECT_TRUE(vector.isApprox(fromOsg(osgVector)));
}

TEST(OsgVectorConversionsTests, Vector4dTest)
{
	Vector4d vector = Vector4d::Random();
	osg::Vec4d osgVector = toOsg(vector);
	EXPECT_TRUE(vector.isApprox(fromOsg(osgVector)));
}
