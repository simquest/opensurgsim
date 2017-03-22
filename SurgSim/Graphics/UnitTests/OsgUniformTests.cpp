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
/// Tests for the OsgUniform class.

#include "SurgSim/Graphics/OsgUniform.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/MathConvert.h"

#include <gtest/gtest.h>

#include <random>

#include <boost/align/aligned_allocator.hpp>

using SurgSim::Math::Vector2f;
using SurgSim::Math::Vector3f;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Matrix22f;
using SurgSim::Math::Matrix33f;
using SurgSim::Math::Matrix44f;
using SurgSim::Math::Matrix22d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix44d;

namespace
{
/// Random number generator, used to generate random values for the tests.
std::default_random_engine generator;
}

namespace SurgSim
{
namespace Graphics
{

/// Constructs an OsgUniform, sets it to the given value, and returns the result of Uniform::get() and the wrapped
/// osg::Uniform::get().
/// \tparam	Type	Uniform's value type
/// \tparam	OsgType	Type stored in the osg::Uniform
template <class Type, class OsgType>
std::pair<Type, OsgType> testUniformConstruction(const Type& value)
{
	
	std::shared_ptr<OsgUniform<Type>> osgUniform(new OsgUniform<Type>("test name"));
	std::shared_ptr<OsgUniformBase> osgUniformBase = osgUniform;
	std::shared_ptr<Uniform<Type>> uniform = osgUniform;
	std::shared_ptr<UniformBase> uniformBase = osgUniform;

	EXPECT_EQ("test name", osgUniformBase->getName());

	uniform->set(value);

	OsgType osgValue;
	EXPECT_TRUE(osgUniformBase->getOsgUniform()->get(osgValue)) <<
			"Failed to get osg::Uniform value. The Uniform type may be wrong!";

	return std::make_pair(uniform->get(), osgValue);
}

template <class Type>
std::pair<Type, boost::any> testAccessible(const Type& value)
{
	std::shared_ptr<OsgUniform<Type>> osgUniform(new OsgUniform<Type>("test name"));

	osgUniform->setValue("Value", value);

	return std::make_pair(osgUniform->get(), osgUniform->getValue("Value"));
}

template <class Type>
Type testYamlSetter(const Type& value)
{
	YAML::Node node = YAML::convert<Type>::encode(value);
	std::shared_ptr<OsgUniform<Type>> osgUniform(new OsgUniform<Type>("test name"));
	osgUniform->set(node);
	return osgUniform->get();
}

template <class Type>
Type testEncodeDecode(const Type& value)
{
	std::shared_ptr<OsgUniform<Type>> osgUniform(new OsgUniform<Type>("test name"));
	osgUniform->set(value);
	std::shared_ptr<OsgUniformBase> base = osgUniform;

	YAML::Node node;

	node = base;

	std::shared_ptr<OsgUniform<Type>> converted;

	converted = std::dynamic_pointer_cast<OsgUniform<Type>>(node.as<std::shared_ptr<OsgUniformBase>>());

	return converted->get();
}

/// Constructs an OsgUniform that stores a vector of values, sets it to the given vector values, and returns the result
/// of Uniform::get() and the wrapped osg::Uniform::get().
/// \tparam	Type	Uniform's value type
/// \tparam	OsgType	Type stored in the osg::Uniform
template <class Type, class OsgType>
std::pair<std::vector<Type>, std::vector<OsgType>> testUniformElementsConstruction(
			const std::vector<Type>& value, size_t numElements)
{
	std::shared_ptr<OsgUniform<std::vector<Type>>> osgUniform =
		std::make_shared<OsgUniform<std::vector<Type>>>("test name", numElements);
	std::shared_ptr<OsgUniformBase> osgUniformBase = osgUniform;
	std::shared_ptr<Uniform<std::vector<Type>>> uniform = osgUniform;
	std::shared_ptr<UniformBase> uniformBase = osgUniform;

	EXPECT_EQ("test name", osgUniformBase->getName());

	uniform->set(value);

	std::vector<OsgType> osgValue;

	for (unsigned int i = 0; i < osgUniformBase->getOsgUniform()->getNumElements(); ++i)
	{
		OsgType element;
		EXPECT_TRUE(osgUniformBase->getOsgUniform()->getElement(i, element)) <<
				"Failed to get osg::Uniform element value. The Uniform type may be wrong!";
		osgValue.push_back(element);
	}

	return std::make_pair(uniform->get(), osgValue);
}

/// Tests OsgUniform with a random floating point type value.
/// \tparam	FloatType	Floating point type (float, double, ...)
/// \param	min	Minimum random value
/// \param	max	Maximum random value
template <class FloatType>
void testUniformFloat(FloatType min, FloatType max)
{
	std::uniform_real_distribution<FloatType> distribution(min, max);
	FloatType value = distribution(generator);
	std::pair<FloatType, FloatType> result = testUniformConstruction<FloatType, FloatType>(value);
	EXPECT_NEAR(value, result.first, Eigen::NumTraits<FloatType>::dummy_precision());
	EXPECT_NEAR(value, result.second, Eigen::NumTraits<FloatType>::dummy_precision());

	auto accessibleResult = testAccessible<FloatType>(value);
	FloatType accessibleValue;
	ASSERT_NO_THROW({boost::any_cast<FloatType>(accessibleResult.second);});
	accessibleValue = boost::any_cast<FloatType>(accessibleResult.second);
	EXPECT_NEAR(value, accessibleValue, Eigen::NumTraits<FloatType>::dummy_precision());

	FloatType nodeValue = 0.0;
	ASSERT_NO_THROW(nodeValue = testYamlSetter<FloatType>(value));
	EXPECT_NEAR(value, nodeValue, Eigen::NumTraits<FloatType>::dummy_precision());

	nodeValue = 0.0;
	ASSERT_NO_THROW(nodeValue = testEncodeDecode<FloatType>(value));
	EXPECT_NEAR(value, nodeValue, Eigen::NumTraits<FloatType>::dummy_precision());
}

/// Tests OsgUniform with a vector of random floating point type values.
/// \tparam	FloatType	Floating point type (float, double, ...)
/// \param	min	Minimum random value
/// \param	max	Maximum random value
/// \param	numElements	Number of elements
template <class FloatType>
void testUniformElementsFloat(FloatType min, FloatType max, size_t numElements)
{
	std::uniform_real_distribution<FloatType> distribution(min, max);
	std::vector<FloatType> elements;
	for (size_t i = 0; i < numElements; ++i)
	{
		elements.push_back(distribution(generator));
	}

	std::pair<std::vector<FloatType>, std::vector<FloatType>> result =
				testUniformElementsConstruction<FloatType, FloatType>(elements, numElements);

	EXPECT_EQ(elements.size(), result.first.size()) << "Number of resulting float-type elements does not match input";
	EXPECT_EQ(elements.size(), result.second.size()) << "Number of resulting OSG-type elements does not match input";

	for (size_t i = 0; i < elements.size(); ++i)
	{
		EXPECT_NEAR(elements[i], result.first[i], Eigen::NumTraits<FloatType>::dummy_precision());
		EXPECT_NEAR(elements[i], result.second[i], Eigen::NumTraits<FloatType>::dummy_precision());
	}
}

/// Tests OsgUniform with a random integer type value.
/// \tparam	IntType	Integer type (int, unsigned int, ...)
/// \param	min	Minimum random value
/// \param	max	Maximum random value
template <class IntType>
void testUniformInt(IntType min, IntType max)
{
	std::uniform_int_distribution<IntType> distribution(min, max);
	IntType value = distribution(generator);
	std::pair<IntType, IntType> result = testUniformConstruction<IntType, IntType>(value);
	EXPECT_EQ(value, result.first);
	EXPECT_EQ(value, result.second);

	auto accessibleResult = testAccessible<IntType>(value);
	IntType accessibleValue;
	ASSERT_NO_THROW({accessibleValue = boost::any_cast<IntType>(accessibleResult.second);});
	EXPECT_EQ(value, accessibleValue);

	IntType nodeValue = 0;
	EXPECT_NO_THROW(nodeValue = testYamlSetter<IntType>(value));
	EXPECT_EQ(value, nodeValue);

	nodeValue = 0;
	ASSERT_NO_THROW(nodeValue = testEncodeDecode<IntType>(value));
	EXPECT_NEAR(value, nodeValue, Eigen::NumTraits<IntType>::dummy_precision());
}

/// Tests OsgUniform with a vector of random integer type values.
/// \tparam	IntType	Integer type (int, unsigned int, ...)
/// \param	min	Minimum random value
/// \param	max	Maximum random value
/// \param	numElements	Number of elements
template <class IntType>
void testUniformElementsInt(IntType min, IntType max, size_t numElements)
{
	std::uniform_int_distribution<IntType> distribution(min, max);
	std::vector<IntType> elements;
	for (size_t i = 0; i < numElements; ++i)
	{
		elements.push_back(distribution(generator));
	}

	std::pair<std::vector<IntType>, std::vector<IntType>> result =
				testUniformElementsConstruction<IntType, IntType>(elements, numElements);

	EXPECT_EQ(elements.size(), result.first.size()) << "Number of resulting int-type elements does not match input";
	EXPECT_EQ(elements.size(), result.second.size()) << "Number of resulting OSG-type elements does not match input";

	for (size_t i = 0; i < elements.size(); ++i)
	{
		EXPECT_EQ(elements[i], result.first[i]);
		EXPECT_EQ(elements[i], result.second[i]);
	}
}

/// Tests OsgUniform with a random Eigen type values.
/// \tparam	Type	Eigen type (Vector2f, Matrix44d, ...)
/// \tparam	OsgType	OSG type which corresponds with the Eigen type (must have a fromOsg() defined for this type)
template <class Type, class OsgType>
void testUniformEigen()
{
	Type value = Type::Random();
	std::pair<Type, OsgType> result = testUniformConstruction<Type, OsgType>(value);
	EXPECT_TRUE(result.first.isApprox(value));
	EXPECT_TRUE(fromOsg(result.second).isApprox(value));

	auto accessibleResult = testAccessible<Type>(value);
	Type accessibleValue;
	ASSERT_NO_THROW({boost::any_cast<Type>(accessibleResult.second);});
	accessibleValue = boost::any_cast<Type>(accessibleResult.second);
	EXPECT_TRUE(value.isApprox(accessibleValue));

	{
		Type nodeValue;
		EXPECT_NO_THROW(nodeValue = testYamlSetter<Type>(value));
		EXPECT_TRUE(value.isApprox(nodeValue));
	}

	{
		Type nodeValue;
		ASSERT_NO_THROW(nodeValue = testEncodeDecode<Type>(value));
		EXPECT_TRUE(value.isApprox(nodeValue));
	}
}

/// Tests OsgUniform with a vector of random Eigen type values.
/// \tparam	Type	Eigen type (Vector2f, Matrix44d, ...)
/// \tparam	OsgType	OSG type which corresponds with the Eigen type (must have a fromOsg() defined for this type)
template <class Type, class OsgType>
void testUniformElementsEigen(size_t numElements)
{
	std::vector<Type> elements;
	for (size_t i = 0; i < numElements; ++i)
	{
		elements.push_back(Type::Random());
	}

	std::pair<std::vector<Type>, std::vector<OsgType>> result =
				testUniformElementsConstruction<Type, OsgType>(elements, numElements);

	EXPECT_EQ(elements.size(), result.first.size()) << "Number of resulting Eigen-type elements does not match input";
	EXPECT_EQ(elements.size(), result.second.size()) << "Number of resulting OSG-type elements does not match input";

	for (size_t i = 0; i < elements.size(); ++i)
	{
		const Type& eigenInput = elements[i];
		const Type& eigenOutput = result.first[i];
		const OsgType& osgOutput = result.second[i];

		EXPECT_TRUE(eigenOutput.isApprox(eigenInput));
		EXPECT_TRUE(fromOsg(osgOutput).isApprox(eigenInput));
	}
}

TEST(OsgUniformTests, FloatTest)
{
	testUniformFloat<float>(-10.0f, 10.0f);
}
TEST(OsgUniformTests, DoubleTest)
{
	testUniformFloat<double>(-10.0, 10.0);
}
TEST(OsgUniformTests, IntTest)
{
	testUniformInt<int>(-10, 10);
}
TEST(OsgUniformTests, UnsignedIntTest)
{
	testUniformInt<unsigned int>(0, 10);
}
TEST(OsgUniformTests, BoolTest)
{
	{
		std::pair<bool, bool> result = testUniformConstruction<bool, bool>(true);
		EXPECT_TRUE(result.first);
		EXPECT_TRUE(result.second);
	}
	{
		std::pair<bool, bool> result = testUniformConstruction<bool, bool>(false);
		EXPECT_FALSE(result.first);
		EXPECT_FALSE(result.second);
	}
}

TEST(OsgUniformTests, Vector2fTest)
{
	testUniformEigen<Vector2f, osg::Vec2f>();
}
TEST(OsgUniformTests, Vector3fTest)
{
	testUniformEigen<Vector3f, osg::Vec3f>();
}
TEST(OsgUniformTests, Vector4fTest)
{
	testUniformEigen<Vector4f, osg::Vec4f>();
}

TEST(OsgUniformTests, Vector2dTest)
{
	testUniformEigen<Vector2d, osg::Vec2d>();
}
TEST(OsgUniformTests, Vector3dTest)
{
	testUniformEigen<Vector3d, osg::Vec3d>();
}
TEST(OsgUniformTests, Vector4dTest)
{
	testUniformEigen<Vector4d, osg::Vec4d>();
}

TEST(OsgUniformTests, Matrix22fTest)
{
	testUniformEigen<Matrix22f, osg::Matrix2>();
}
TEST(OsgUniformTests, Matrix33fTest)
{
	testUniformEigen<Matrix33f, osg::Matrix3>();
}
TEST(OsgUniformTests, Matrix44fTest)
{
	testUniformEigen<Matrix44f, osg::Matrixf>();
}

TEST(OsgUniformTests, Matrix22dTest)
{
	testUniformEigen<Matrix22d, osg::Matrix2d>();
}
TEST(OsgUniformTests, Matrix33dTest)
{
	testUniformEigen<Matrix33d, osg::Matrix3d>();
}
TEST(OsgUniformTests, Matrix44dTest)
{
	testUniformEigen<Matrix44d, osg::Matrixd>();
}

TEST(OsgUniformTests, FloatElementsTest)
{
	testUniformElementsFloat<float>(-10.0f, 10.0f, 10);
}
TEST(OsgUniformTests, DoubleElementsTest)
{
	testUniformElementsFloat<double>(-10.0, 10.0, 10);
}
TEST(OsgUniformTests, IntElementsTest)
{
	testUniformElementsInt<int>(-10, 10, 10);
}
TEST(OsgUniformTests, UnsignedIntElementsTest)
{
	testUniformElementsInt<unsigned int>(0, 10, 10);
}
TEST(OsgUniformTests, BoolElementsTest)
{
	std::vector<bool> elements;
	for (size_t i = 0; i < 10; ++i)
	{
		elements.push_back(i % 2 == 0);
	}

	std::pair<std::vector<bool>, std::vector<bool>> result =
				testUniformElementsConstruction<bool, bool>(elements, 10);

	EXPECT_EQ(elements.size(), result.first.size()) << "Number of resulting bool-type elements does not match input";
	EXPECT_EQ(elements.size(), result.second.size()) << "Number of resulting OSG-type elements does not match input";

	for (size_t i = 0; i < elements.size(); ++i)
	{
		EXPECT_EQ(elements[i], result.first[i]);
		EXPECT_EQ(elements[i], result.second[i]);
	}
}

TEST(OsgUniformTests, Vector2fElementsTest)
{
	testUniformElementsEigen<Vector2f, osg::Vec2f>(10);
}
TEST(OsgUniformTests, Vector3fElementsTest)
{
	testUniformElementsEigen<Vector3f, osg::Vec3f>(10);
}
TEST(OsgUniformTests, Vector4fElementsTest)
{
	testUniformElementsEigen<Vector4f, osg::Vec4f>(10);
}

TEST(OsgUniformTests, Vector2dElementsTest)
{
	testUniformElementsEigen<Vector2d, osg::Vec2d>(10);
}
TEST(OsgUniformTests, Vector3dElementsTest)
{
	testUniformElementsEigen<Vector3d, osg::Vec3d>(10);
}
TEST(OsgUniformTests, Vector4dElementsTest)
{
	testUniformElementsEigen<Vector4d, osg::Vec4d>(10);
}

TEST(OsgUniformTests, Matrix22fElementsTest)
{
	testUniformElementsEigen<Matrix22f, osg::Matrix2>(10);
}
TEST(OsgUniformTests, Matrix33fElementsTest)
{
	testUniformElementsEigen<Matrix33f, osg::Matrix3>(10);
}
TEST(OsgUniformTests, Matrix44fElementsTest)
{
	testUniformElementsEigen<Matrix44f, osg::Matrixf>(10);
}

TEST(OsgUniformTests, Matrix22dElementsTest)
{
	testUniformElementsEigen<Matrix22d, osg::Matrix2d>(10);
}
TEST(OsgUniformTests, Matrix33dElementsTest)
{
	testUniformElementsEigen<Matrix33d, osg::Matrix3d>(10);
}
TEST(OsgUniformTests, Matrix44dElementsTest)
{
	testUniformElementsEigen<Matrix44d, osg::Matrixd>(10);
}

}  // namespace Graphics
}  // namespace SurgSim
