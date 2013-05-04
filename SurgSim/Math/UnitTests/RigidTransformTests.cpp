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
/// Tests that exercise the functionality of our rigid transform typedefs, which
/// come straight from Eigen.

#include <Eigen/Geometry>
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "gtest/gtest.h"

template <class T>
class RigidTransformTestBase : public testing::Test
{
public:
	typedef T RigidTransform;
};


template <class T>
class RigidTransform3Tests : public RigidTransformTestBase<T>
{
};

typedef ::testing::Types<SurgSim::Math::RigidTransform3d,
						 SurgSim::Math::RigidTransform3f> RigidTransform3Variants;
TYPED_TEST_CASE(RigidTransform3Tests, RigidTransform3Variants);


template <class T>
class AllRigidTransformTests : public RigidTransformTestBase<T>
{
};

typedef ::testing::Types<SurgSim::Math::RigidTransform2d,
						 SurgSim::Math::RigidTransform2f,
						 SurgSim::Math::RigidTransform3d,
						 SurgSim::Math::RigidTransform3f> AllRigidTransformVariants;
TYPED_TEST_CASE(AllRigidTransformTests, AllRigidTransformVariants);


/// Test that rigid transforms can be constructed
TYPED_TEST(AllRigidTransformTests, CanConstruct)
{
	typename TestFixture::RigidTransform transform;
}

/// Test makeRigidTransform using a rotation matrix and a translation
TYPED_TEST(AllRigidTransformTests, MakeRigidTransform)
{
	typedef typename TestFixture::RigidTransform RigidTransform;
	typedef typename RigidTransform::LinearMatrixType RotationMatrix;
	typedef typename RigidTransform::TranslationType::VectorType Translation;
	const int DIM = RigidTransform::Dim;

	RotationMatrix rotationMatrix = RotationMatrix::Identity();
	rotationMatrix.row(0).swap(rotationMatrix.row(DIM-1));
	Translation translation(Translation::Random());
	RigidTransform transform = SurgSim::Math::makeRigidTransform(rotationMatrix, translation);

	typename RigidTransform::MatrixType matrix = transform.matrix();
	EXPECT_TRUE( (rotationMatrix.isApprox(matrix.block(0,0,DIM,DIM), 1e-6)) )
		<< "Rotation part of transform is not properly set";
	EXPECT_TRUE( (translation.isApprox(matrix.block(0,DIM,DIM,1), 1e-6)) )
		<< "Translation part of transform is not properly set";
	EXPECT_NEAR(1.0, matrix(DIM,DIM), 1e-6)
		<< "Transform matrix is not 1.0 in bottom right corner";
	EXPECT_TRUE( (matrix.block(DIM,0,1,DIM).isApproxToConstant(0.0, 1e-6)) )
		<< "Bottom row of matrix is not all zeros (except for last column).";
}

/// Test makeRigidTransform using a quaternion and a translation
TYPED_TEST(RigidTransform3Tests, MakeRigidTransformWithQuaternion)
{
	typedef typename TestFixture::RigidTransform RigidTransform;
	typedef typename RigidTransform::TranslationType::VectorType Translation;
	typedef typename RigidTransform::Scalar Scalar;
	typedef typename Eigen::Quaternion<Scalar> Quaternion;
	typedef typename RigidTransform::LinearMatrixType RotationMatrix;
	const int DIM = RigidTransform::Dim;

	Translation translation = Translation::Random();
	Quaternion quaternion(0.0, 1.0, 0.0, 0.0);
	RotationMatrix quaternionRotationMatrix;
	quaternionRotationMatrix << 1.0, 0.0, 0.0,
								0.0,-1.0, 0.0,
								0.0, 0.0,-1.0;
	RigidTransform transform = SurgSim::Math::makeRigidTransform(quaternion, translation);

	typename RigidTransform::MatrixType matrix = transform.matrix();
	EXPECT_TRUE( (quaternionRotationMatrix.isApprox(matrix.block(0,0,DIM,DIM), 1e-6)) )
		<< "Rotation part of transform is not properly set";
	EXPECT_TRUE( (translation.isApprox(matrix.block(0,DIM,DIM,1), 1e-6)) )
		<< "Translation part of transform is not properly set";
	EXPECT_NEAR(1.0, matrix(DIM,DIM), 1e-6)
		<< "Transform matrix is not 1.0 in bottom right corner";
	EXPECT_TRUE( (matrix.block(DIM,0,1,DIM).isApproxToConstant(0.0, 1e-6)) )
		<< "Bottom row of matrix is not all zeros (except for last column).";
}
