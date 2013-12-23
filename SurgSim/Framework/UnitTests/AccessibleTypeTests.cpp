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

#include <gtest/gtest.h>
#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Serialize/MathConvert.h"


// We need ALL of the math types
using namespace SurgSim::Math; //NOLINT

namespace SurgSim
{
namespace Framework
{

template <class T>
class BaseTest : public testing::Test
{
public:
	typedef T Scalar;
};

template <class T>
class ScalarTest : public BaseTest<T>
{
};

typedef ::testing::Types<int, size_t, double, float> ScalarTypes;
TYPED_TEST_CASE(ScalarTest, ScalarTypes);

template <class T>
class VectorTest: public BaseTest<typename T::Scalar>
{
public:
	typedef T Vector;
};

typedef ::testing::Types<SurgSim::Math::Vector2d,
						 SurgSim::Math::Vector2f,
						 SurgSim::Math::Vector3d,
						 SurgSim::Math::Vector3f,
						 SurgSim::Math::Vector4d,
						 SurgSim::Math::Vector4f> VectorTypes;

TYPED_TEST_CASE(VectorTest, VectorTypes);

template <class T>
class QuaternionTest : public BaseTest<typename T::Scalar>
{
public:
	typedef T Quaternion;
};

typedef ::testing::Types<SurgSim::Math::Quaterniond, SurgSim::Math::Quaternionf> QuaternionTypes;
TYPED_TEST_CASE(QuaternionTest, QuaternionTypes);

template <class T>
class RigidTransformTest : public BaseTest<typename T::Scalar>
{
public:
	typedef T RigidTransform;
};
typedef ::testing::Types<SurgSim::Math::RigidTransform2f,
						 SurgSim::Math::RigidTransform2d,
						 SurgSim::Math::RigidTransform3f,
						 SurgSim::Math::RigidTransform3d> RigidTransformTypes;

TYPED_TEST_CASE(RigidTransformTest, RigidTransformTypes);

template <class T>
class Testable : public Accessible
{
public:
	Testable()
	{
		setAccessors("property",
			std::bind(&Testable::getProperty, this),
			std::bind(&Testable::setProperty, this, std::bind(SurgSim::Framework::convert<T>,std::placeholders::_1)));
		setSerializable("property",
			std::bind(&YAML::convert<T>::encode, std::bind(&Testable::getProperty, this)),
			std::bind(&Testable::setProperty, this, std::bind(&YAML::Node::as<T>,std::placeholders::_1)));
	}

	T property;

	void setProperty(T value) {property = value;}
	T getProperty() {return property;}
};

template <typename T>
std::pair<T, T> testProperty(T a, T b)
{
	Testable<T> test;
	std::pair<T, T> result;
	test.property = a;

	test.setValue("property", b);
	result.first = test.property;

	test.getValue("property", &result.second);

	return result;
}

template <typename T>
std::pair<T, T> testEncodeDecode(T a, T b)
{
	Testable<T> test;
	std::pair<T, T> result;
	test.property = a;

	YAML::Node node;
	node["property"] = b;
	test.decode(node);

	result.first = test.property;

	YAML::Node resultNode = test.encode();
	result.second = resultNode["property"].as<T>();

	return result;
}

TYPED_TEST(ScalarTest, Accessible)
{
	typedef typename TestFixture::Scalar Scalar;

	Scalar initialValue = static_cast<Scalar>(1);
	Scalar newValue = static_cast<Scalar>(2);

	std::pair<Scalar,Scalar> result = testProperty(initialValue, newValue);
	EXPECT_NEAR(newValue, result.first, 1e-6);
	EXPECT_NEAR(newValue, result.second, 1e-6);

	result = testEncodeDecode(initialValue, newValue);
	EXPECT_NEAR(newValue, result.first, 1e-6);
	EXPECT_NEAR(newValue, result.second, 1e-6);
}


TYPED_TEST(VectorTest, Accessible)
{
	typedef typename TestFixture::Scalar Scalar;
	typedef typename TestFixture::Vector Vector;

	Vector initialValue;
	Vector newValue;

	for (int i = 0; i < initialValue.size(); ++i)
	{
		initialValue[i] = static_cast<Scalar>(i);
		newValue[i] = static_cast<Scalar>(i*2);
	}

	std::pair<Vector, Vector> result = testProperty<TypeParam >(initialValue, newValue);
	EXPECT_TRUE(newValue.isApprox(result.first));
	EXPECT_TRUE(newValue.isApprox(result.second));

	result = testEncodeDecode<TypeParam>(initialValue, newValue);
	EXPECT_TRUE(newValue.isApprox(result.first));
	EXPECT_TRUE(newValue.isApprox(result.second));
}

TYPED_TEST(QuaternionTest, Accessible)
{
	typedef typename TestFixture::Scalar Scalar;
	typedef typename TestFixture::Quaternion Quaternion;

	Quaternion initialValue(static_cast<Scalar>(1.0),
							static_cast<Scalar>(2.0),
							static_cast<Scalar>(3.0),
							static_cast<Scalar>(4.0));
	Quaternion newValue(static_cast<Scalar>(5.0),
						static_cast<Scalar>(6.0),
						static_cast<Scalar>(7.0),
						static_cast<Scalar>(8.0));

	initialValue.normalize();
	newValue.normalize();

	std::pair<Quaternion, Quaternion> result = testProperty<TypeParam >(initialValue, newValue);
	EXPECT_TRUE(newValue.isApprox(result.first));
	EXPECT_TRUE(newValue.isApprox(result.second));

	result = testEncodeDecode<Quaternion>(initialValue, newValue);
	EXPECT_TRUE(newValue.isApprox(result.first));
	EXPECT_TRUE(newValue.isApprox(result.second));
}

TYPED_TEST(RigidTransformTest, Accessible)
{
	typedef typename TestFixture::Scalar Scalar;
	typedef typename TestFixture::RigidTransform RigidTransform;

	typename RigidTransform::MatrixType initialMatrix;
	typename RigidTransform::MatrixType newMatrix;

	for (int i = 0; i < initialMatrix.size(); ++i)
	{
		initialMatrix(i) = static_cast<Scalar>(i);
		newMatrix(i) = static_cast<Scalar>(i*2);
	}

	RigidTransform initialValue(initialMatrix);
	RigidTransform newValue(newMatrix);

	std::pair<RigidTransform, RigidTransform> result = testProperty<RigidTransform >(initialValue, newValue);
	EXPECT_TRUE(newValue.isApprox(result.first));
	EXPECT_TRUE(newValue.isApprox(result.second));

	result = testEncodeDecode<RigidTransform>(initialValue, newValue);
	EXPECT_TRUE(newValue.isApprox(result.first));
	EXPECT_TRUE(newValue.isApprox(result.second));
}

}
}
