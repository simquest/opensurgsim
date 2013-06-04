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
/// Tests for conversions to and from OSG matrix types

#include <SurgSim/Graphics/OsgMatrixConversions.h>
#include <SurgSim/Graphics/OsgVectorConversions.h>

#include <gtest/gtest.h>

using SurgSim::Graphics::fromOsg;
using SurgSim::Graphics::toOsg;
using SurgSim::Math::Matrix22f;
using SurgSim::Math::Matrix22d;
using SurgSim::Math::Matrix33f;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix44f;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Vector2f;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3f;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector4d;


TEST(OsgMatrixConversionsTests, Matrix22fTest)
{
	Matrix22f matrix = Matrix22f::Random();
	osg::Matrix2 osgMatrix = toOsg(matrix);
	EXPECT_TRUE(matrix.isApprox(fromOsg<float>(osgMatrix)));
}

TEST(OsgMatrixConversionsTests, Matrix22dTest)
{
	Matrix22d matrix = Matrix22d::Random();
	osg::Matrix2 osgMatrix = toOsg(matrix);
	/// OSG only stores the values as floats, so precision is lost: use float precision for comparison
	EXPECT_TRUE(matrix.isApprox(fromOsg<double>(osgMatrix), Eigen::NumTraits<float>::dummy_precision()));
}

TEST(OsgMatrixConversionsTests, Matrix33fTest)
{
	Matrix33f matrix = Matrix33f::Random();
	osg::Matrix3 osgMatrix = toOsg(matrix);
	EXPECT_TRUE(matrix.isApprox(fromOsg<float>(osgMatrix)));
}

TEST(OsgMatrixConversionsTests, Matrix33dTest)
{
	Matrix33d matrix = Matrix33d::Random();
	osg::Matrix3 osgMatrix = toOsg(matrix);
	/// OSG only stores the values as floats, so precision is lost: use float precision for comparison
	EXPECT_TRUE(matrix.isApprox(fromOsg<double>(osgMatrix), Eigen::NumTraits<float>::dummy_precision()));
}

TEST(OsgMatrixConversionsTests, Matrix44fConversionTest)
{
	Matrix44f matrix = Matrix44f::Random();
	osg::Matrixf osgMatrix = toOsg(matrix);
	EXPECT_TRUE(matrix.isApprox(fromOsg(osgMatrix)));
}

TEST(OsgMatrixConversionsTests, Matrix44fMultiplicationTest)
{
	Matrix44f matrix = Matrix44f::Random();
	Vector4f vector = Vector4f::Random();

	osg::Matrixf osgMatrix = toOsg(matrix);
	osg::Vec4f osgVector = toOsg(vector);

	/// Multiply with Eigen
	Vector4f result = matrix * vector;
	/// Multiply with OSG
	osg::Vec4f osgResult = osgVector * osgMatrix;

	/// Compare the two results
	EXPECT_TRUE(result.isApprox(fromOsg(osgResult)));
}

TEST(OsgMatrixConversionsTests, Matrix44dTest)
{
	Matrix44d matrix = Matrix44d::Random();
	osg::Matrixd osgMatrix = toOsg(matrix);
	EXPECT_TRUE(matrix.isApprox(fromOsg(osgMatrix)));
}

TEST(OsgMatrixConversionsTests, Matrix44dMultiplicationTest)
{
	Matrix44d matrix = Matrix44d::Random();
	Vector4d vector = Vector4d::Random();

	osg::Matrixd osgMatrix = toOsg(matrix);
	osg::Vec4d osgVector = toOsg(vector);

	/// Multiply with Eigen
	Vector4d result = matrix * vector;
	/// Multiply with OSG
	osg::Vec4d osgResult = osgVector * osgMatrix;

	/// Compare the two results
	EXPECT_TRUE(result.isApprox(fromOsg(osgResult)));
}