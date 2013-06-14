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
/// Tests that exercise the functionality of our rigid transform typedefs, which
/// come straight from Eigen.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Quaternion.h>
#include "gtest/gtest.h"

template <class T>
class RigidTransformTestBase : public testing::Test
{
public:
	typedef T RigidTransform;
	typedef typename T::Scalar Scalar;
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

/// Test rigid transforms interpolation
TYPED_TEST(AllRigidTransformTests, Interpolation)
{
	typedef typename TestFixture::RigidTransform Transform;
	typedef typename TestFixture::Scalar T;

	// Eigen does something fancy in the slerp. If your 2 quaternions have an angle bigger than PI
	// it will interpolate between q0 and -q1. Which means that interpolating at time 1 will not give you q1.
	const T angle = static_cast<T>(M_PI * 0.99);
	const Eigen::Matrix<T, 3, 1> axis (0.0, 1.0, 0.0);
	Eigen::Quaternion<T> rotation = SurgSim::Math::makeRotationQuaternion(angle, axis);

	Eigen::Quaternion<T> q0(Eigen::Matrix<T,4,1>::Random());
	q0.normalize();
	Eigen::Quaternion<T> q1 = rotation * q0;
	q1.normalize();
	Eigen::Matrix<T, 3, 1> t0(Eigen::Matrix<T,3,1>::Random());
	Eigen::Matrix<T, 3, 1> t1(Eigen::Matrix<T,3,1>::Random());

	Eigen::Transform<T, 3, Eigen::Isometry> transform0 = SurgSim::Math::makeRigidTransform(q0, t0);
	Eigen::Transform<T, 3, Eigen::Isometry> transform1 = SurgSim::Math::makeRigidTransform(q1, t1);
	EXPECT_TRUE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.0)).isApprox(transform0));
	EXPECT_TRUE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(1.0)).isApprox(transform1));

	EXPECT_FALSE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.234)).isApprox(transform0));
	EXPECT_FALSE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.234)).isApprox(transform1));

	EXPECT_FALSE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.5)).isApprox(transform0));
	EXPECT_FALSE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.5)).isApprox(transform1));
	// At t=0.5, the inteprolation should return (q0 + q1)/2 normalized
	// c.f. http://en.wikipedia.org/wiki/Slerp
	Eigen::Quaternion<T> qHalf( (q0.coeffs() + q1.coeffs()) * 0.5);
	qHalf.normalize();
	Eigen::Matrix<T, 3, 1> tHalf = (t0 + t1) * 0.5;
	Eigen::Transform<T, 3, Eigen::Isometry> transformHalf = SurgSim::Math::makeRigidTransform(qHalf, tHalf);
	EXPECT_TRUE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.5)).isApprox(transformHalf));

	EXPECT_FALSE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.839)).isApprox(transform0));
	EXPECT_FALSE(SurgSim::Math::interpolateRigidTransform(transform0, transform1, static_cast<T>(0.839)).isApprox(transform1));
}
