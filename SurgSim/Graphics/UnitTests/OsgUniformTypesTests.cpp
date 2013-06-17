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
/// Tests for the OsgUniformTypes functions to make sure they return the correct enum values.

#include <SurgSim/Graphics/OsgUniformTypes.h>

#include <gtest/gtest.h>

#include <random>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgUniformTypesTests, FloatTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT, getOsgUniformType<float>());
}
TEST(OsgUniformTypesTests, DoubleTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE, getOsgUniformType<double>());
}
TEST(OsgUniformTypesTests, IntTest)
{
	EXPECT_EQ(osg::Uniform::INT, getOsgUniformType<int>());
}
TEST(OsgUniformTypesTests, UnsignedIntTest)
{
	EXPECT_EQ(osg::Uniform::UNSIGNED_INT, getOsgUniformType<unsigned int>());
}
TEST(OsgUniformTypesTests, BoolTest)
{
	EXPECT_EQ(osg::Uniform::BOOL, getOsgUniformType<bool>());
}

TEST(OsgUniformTypesTests, Vector2fTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT_VEC2, getOsgUniformType<SurgSim::Math::Vector2f>());
}
TEST(OsgUniformTypesTests, Vector3fTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT_VEC3, getOsgUniformType<SurgSim::Math::Vector3f>());
}
TEST(OsgUniformTypesTests, Vector4fTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT_VEC4, getOsgUniformType<SurgSim::Math::Vector4f>());
}

TEST(OsgUniformTypesTests, Vector2dTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE_VEC2, getOsgUniformType<SurgSim::Math::Vector2d>());
}
TEST(OsgUniformTypesTests, Vector3dTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE_VEC3, getOsgUniformType<SurgSim::Math::Vector3d>());
}
TEST(OsgUniformTypesTests, Vector4dTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE_VEC4, getOsgUniformType<SurgSim::Math::Vector4d>());
}

TEST(OsgUniformTypesTests, Matrix22fTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT_MAT2, getOsgUniformType<SurgSim::Math::Matrix22f>());
}
TEST(OsgUniformTypesTests, Matrix33fTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT_MAT3, getOsgUniformType<SurgSim::Math::Matrix33f>());
}
TEST(OsgUniformTypesTests, Matrix44fTest)
{
	EXPECT_EQ(osg::Uniform::FLOAT_MAT4, getOsgUniformType<SurgSim::Math::Matrix44f>());
}

TEST(OsgUniformTypesTests, Matrix22dTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE_MAT2, getOsgUniformType<SurgSim::Math::Matrix22d>());
}
TEST(OsgUniformTypesTests, Matrix33dTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE_MAT3, getOsgUniformType<SurgSim::Math::Matrix33d>());
}
TEST(OsgUniformTypesTests, Matrix44dTest)
{
	EXPECT_EQ(osg::Uniform::DOUBLE_MAT4, getOsgUniformType<SurgSim::Math::Matrix44d>());
}

}  // namespace Graphics
}  // namespace SurgSim