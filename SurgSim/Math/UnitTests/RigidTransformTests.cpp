// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/MathConvert.h"
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
	using SurgSim::Math::makeRigidTransform;

	typedef typename TestFixture::Scalar T;
	typedef Eigen::Quaternion<T> Quaternion;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> Transform;
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	for (size_t numLoop = 0; numLoop < 100; numLoop++)
	{
		Quaternion q0(Eigen::Matrix<T, 4, 1>::Random());
		Quaternion q1(Eigen::Matrix<T, 4, 1>::Random());
		q0.normalize();
		q1.normalize();

		Vector3 t0(Vector3::Random());
		Vector3 t1(Vector3::Random());

		Transform transform0 = makeRigidTransform(q0, t0);
		Transform transform1 = makeRigidTransform(q1, t1);
		{
			Transform transform = SurgSim::Math::interpolate(transform0, transform1, static_cast<T>(0.0));
			EXPECT_TRUE(transform.isApprox(transform0));
		}
		{
			Transform transform = SurgSim::Math::interpolate(transform0, transform1, static_cast<T>(1.0));
			EXPECT_TRUE(transform.isApprox(transform1));
		}

		{
			Transform transform = SurgSim::Math::interpolate(transform0, transform1, static_cast<T>(0.234));
			EXPECT_FALSE(transform.isApprox(transform0));
			EXPECT_FALSE(transform.isApprox(transform1));
		}

		{
			Transform transform = SurgSim::Math::interpolate(transform0, transform1, static_cast<T>(0.5));
			EXPECT_FALSE(transform.isApprox(transform0));
			EXPECT_FALSE(transform.isApprox(transform1));

			// At t=0.5, the rotation interpolation should return (q0 + q1)/2 normalized
			// c.f. http://en.wikipedia.org/wiki/Slerp
			// If the quaternions are over PI angle, the slerp will interpolate between q0 and -q1
			// in this case, the interpolation is (q0 - q1)/2 normalized
			// From our specification, both quaternions could be considered negative, so we extend
			// the tests to these possibilities as well:
			// (-q0 + q1) / 2 normalized
			// (-q0 - q1) / 2 normalized
			Quaternion qHalf0((q0.coeffs() + q1.coeffs()) * 0.5);
			Quaternion qHalf1((q0.coeffs() - q1.coeffs()) * 0.5);
			Quaternion qHalf2((-q0.coeffs() + q1.coeffs()) * 0.5);
			Quaternion qHalf3((-q0.coeffs() - q1.coeffs()) * 0.5);
			qHalf0.normalize();
			qHalf1.normalize();
			qHalf2.normalize();
			qHalf3.normalize();

			Vector3 tHalf = (t0 + t1) * 0.5;
			Transform transformHalf0 = makeRigidTransform(qHalf0, tHalf);
			Transform transformHalf1 = makeRigidTransform(qHalf1, tHalf);
			Transform transformHalf2 = makeRigidTransform(qHalf2, tHalf);
			Transform transformHalf3 = makeRigidTransform(qHalf3, tHalf);
			EXPECT_TRUE(transform.isApprox(transformHalf0) || transform.isApprox(transformHalf1) ||
						transform.isApprox(transformHalf2) || transform.isApprox(transformHalf3));
		}

		{
			Transform transform = SurgSim::Math::interpolate(transform0, transform1, static_cast<T>(0.839));
			EXPECT_FALSE(transform.isApprox(transform0));
			EXPECT_FALSE(transform.isApprox(transform1));
		}
	}
}

TYPED_TEST(AllRigidTransformTests, MakeLookAt)
{
	typedef typename TestFixture::Scalar T;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> Transform;

	typedef Eigen::Matrix<T, 3, 1> Vector3;
	typedef Eigen::Matrix<T, 4, 1> Vector4;

	Vector3 origin(0.0, 0.0, 0.0);
	Vector3 eye(10.0, 10.0, 10.0);
	Vector3 up(0.0, 1.0, 0.0);

	Vector4 center4(0.0, 0.0, 0.0, 1.0);

	// This follows the OpenGl convention for the camera view matrix transform, any axis would do see
	// the documentation for makeRigidTransform and gluLookAt()
	Vector4 direction4(0.0, 0.0, -1.0, 1.0);
	Vector4 eye4(10.0, 10.0, 10.0, 1.0);

	Transform transform = SurgSim::Math::makeRigidTransform(eye, origin, up);

	EXPECT_TRUE(eye4.isApprox(transform * center4));

	Vector4 transformed = transform * direction4;

	Vector3 direction3(transformed[0], transformed[1], transformed[2]);
	EXPECT_TRUE(eye.normalized().isApprox(direction3.normalized()));
}

// Test conversion to and from yaml node
TYPED_TEST(AllRigidTransformTests, YamlConvert)
{
	using SurgSim::Math::makeRigidTransform;

	typedef typename TestFixture::Scalar T;
	typedef Eigen::Quaternion<T> Quaternion;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> Transform;
	typedef Eigen::Matrix<T, 3, 1> Vector3;

	const T inputValues[4] = {1.1f, 2.2f, 3.3f, 4.4f};

	Quaternion quaternion(inputValues);
	quaternion.normalize();

	Vector3 translation(inputValues);

	Transform transform = makeRigidTransform(quaternion, translation);

	YAML::Node node;

	ASSERT_NO_THROW(node = transform);

	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(2u, node.size());

	Transform expected;

	ASSERT_NO_THROW(expected = node.as<Transform>());
	EXPECT_TRUE(transform.isApprox(expected));
}
