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
/// Tests for the Polynomial functions.

#include <array>

#include <gtest/gtest.h>

#include "SurgSim/Math/Polynomial.h"

namespace SurgSim
{

namespace Math
{

namespace
{
double epsilon = 1.0e-10;
}

class PolynomialUtilityTests : public ::testing::Test
{
};

template <typename T>
class PolynomialTests : public ::testing::Test
{
public:

	void initializeConstructor(Polynomial<double, 0>* poly)
	{
		typedef Polynomial<double, 0> PolynomialType;
		EXPECT_NO_THROW(PolynomialType polyNew);
		EXPECT_NO_THROW(PolynomialType polyNew(7.0));
		PolynomialType polyNew(1.0);
		*poly = polyNew;
	}

	void initializeConstructor(Polynomial<double, 1>* poly)
	{
		typedef Polynomial<double, 1> PolynomialType;
		EXPECT_NO_THROW(PolynomialType polyNew);
		EXPECT_NO_THROW(PolynomialType polyNew(7.0, 8));
		PolynomialType polyNew(1.0, 2.0);
		*poly = polyNew;
	}

	void initializeConstructor(Polynomial<double, 2>* poly)
	{
		typedef Polynomial<double, 2> PolynomialType;
		EXPECT_NO_THROW(PolynomialType polyNew);
		EXPECT_NO_THROW(PolynomialType polyNew(7.0, 8.0, 9.0));
		PolynomialType polyNew(1.0, 2.0, 3.0);
		*poly = polyNew;
	}

	void initializeConstructor(Polynomial<double, 3>* poly)
	{
		typedef Polynomial<double, 3> PolynomialType;
		EXPECT_NO_THROW(PolynomialType polyNew);
		EXPECT_NO_THROW(PolynomialType polyNew(7.0, 8.0, 9.0, 10.0));
		PolynomialType polyNew(1.0, 2.0, 3.0, 4.0);
		*poly = polyNew;
	}

	template <int degree>
	void setPolynomialFromOffset(int offset, Polynomial<double, degree>* poly)
	{
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			(*poly)[counter] = static_cast<double>(counter + offset);
		}
	}

	template <int degree>
	void setPolynomialToSmallValue(Polynomial<double, degree>* poly)
	{
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			(*poly)[counter] = epsilon / 2.0;
		}
	}

	template <int degree>
	void checkPolynomialConstructor(Polynomial<double, degree>* poly)
	{
		// Check get ... up to degree
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			EXPECT_DOUBLE_EQ(counter + 1.0, poly->getCoefficient(counter));
			EXPECT_DOUBLE_EQ(counter + 1.0, (*poly)[counter]);
		}

		// Check get ... beyond degree
		for (size_t counter = degree + 1; counter < 5; ++counter)
		{
			EXPECT_DOUBLE_EQ(0.0, poly->getCoefficient(counter));
			EXPECT_THROW((*poly)[counter], SurgSim::Framework::AssertionFailure);
		}

		// Check set up to degree
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			poly->setCoefficient(counter, static_cast<double>(counter));
			EXPECT_DOUBLE_EQ(static_cast<double>(counter), poly->getCoefficient(counter));
			(*poly)[counter] = static_cast<double>(counter + 1);
			EXPECT_DOUBLE_EQ(static_cast<double>(counter + 1), poly->getCoefficient(counter));
		}

		// Check set beyond degree
		EXPECT_THROW(poly->setCoefficient(degree + 1, 12.0), SurgSim::Framework::AssertionFailure);
		EXPECT_THROW((*poly)[degree + 1] = 12, SurgSim::Framework::AssertionFailure);
	}

	template <int degree>
	void checkPolynomialArithmetic(const Polynomial<double, degree>& poly1,
								   const Polynomial<double, degree>& poly2)
	{
		Polynomial<double, degree> scratch;

		// Check evaluate ...
		double evaluate = static_cast<double>(degree + 1);
		for (int counter = degree - 1; counter >= 0; --counter)
		{
			evaluate = (0.5 * evaluate) + counter + 1;
		}
		EXPECT_NEAR(evaluate, poly1.evaluate(0.5), epsilon);

		// Check negation ...
		scratch = -poly1;
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			EXPECT_DOUBLE_EQ(-1 * (static_cast<double>(counter) + 1.0), scratch.getCoefficient(counter));
			EXPECT_DOUBLE_EQ(-1 * (static_cast<double>(counter) + 1.0), scratch[counter]);
		}

		// Check add ...
		scratch = poly1 + poly2;
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			EXPECT_DOUBLE_EQ(2 * static_cast<double>(counter) + 3.0, scratch.getCoefficient(counter));
			EXPECT_DOUBLE_EQ(2 * static_cast<double>(counter) + 3.0, scratch[counter]);
		}

		scratch = poly1;
		scratch += poly2;
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			EXPECT_DOUBLE_EQ(2 * static_cast<double>(counter) + 3.0, scratch.getCoefficient(counter));
			EXPECT_DOUBLE_EQ(2 * static_cast<double>(counter) + 3.0, scratch[counter]);
		}

		// Check subtract ...
		scratch = poly1 - poly2;
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			EXPECT_DOUBLE_EQ(-1.0, scratch.getCoefficient(counter));
			EXPECT_DOUBLE_EQ(-1.0, scratch[counter]);
		}

		scratch = poly1;
		scratch -= poly2;
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			EXPECT_DOUBLE_EQ(-1.0, scratch.getCoefficient(counter));
			EXPECT_DOUBLE_EQ(-1.0, scratch[counter]);
		}

		// Check isApprox ...
		scratch = poly1;
		for (size_t counter = 0; counter <= degree; ++counter)
		{
			scratch[counter] += epsilon / 2.0;
		}
		EXPECT_TRUE(scratch.isApprox(poly1, epsilon));
		scratch[0] = poly1[0] + 2.0 * epsilon;
		for (size_t counter = 0; counter < degree; ++counter)
		{
			EXPECT_FALSE(scratch.isApprox(poly1, epsilon));
			scratch[counter] = poly1[counter] + epsilon / 2.0;
			scratch[counter + 1] = poly1[counter + 1] + 2.0 * epsilon;
		}
		EXPECT_FALSE(scratch.isApprox(poly1, epsilon));
	}

	template <int degree>
	void checkPolynomialDerivative(const Polynomial<double, degree>& poly1)
	{
		if (degree != 0)
		{
			// Check derivative ...
			auto smallScratch = poly1.derivative();
			for (size_t counter = 0; counter < degree; ++counter)
			{
				EXPECT_DOUBLE_EQ((counter + 2.0) * (counter + 1.0), smallScratch.getCoefficient(counter));
				EXPECT_DOUBLE_EQ((counter + 2.0) * (counter + 1.0), smallScratch[counter]);
			}
			EXPECT_DOUBLE_EQ(0.0, smallScratch.getCoefficient(degree));
			EXPECT_THROW(smallScratch[degree], SurgSim::Framework::AssertionFailure);
		}
	}

	template <int degree>
	void checkIsNearZero(Polynomial<double, degree>* poly1)
	{
		EXPECT_TRUE(poly1->isNearZero(epsilon));
		for (size_t counter = 0; counter <= degree; counter++)
		{
			poly1->setCoefficient(counter, 2 * epsilon);
			EXPECT_FALSE(poly1->isNearZero(epsilon));
			poly1->setCoefficient(counter, epsilon / 2.0);
			EXPECT_TRUE(poly1->isNearZero(epsilon));
			(*poly1)[counter] = 2 * epsilon;
			EXPECT_FALSE(poly1->isNearZero(epsilon));
			(*poly1)[counter] = epsilon / 2.0;
			EXPECT_TRUE(poly1->isNearZero(epsilon));
		}
	}
};

template <typename T>
class PolynomialDerivativeTests : public PolynomialTests<T>
{
};

typedef ::testing::Types<Polynomial<double, 0>,
		Polynomial<double, 1>,
		Polynomial<double, 2>,
		Polynomial<double, 3>> PolynomialTypes;

typedef ::testing::Types <
Polynomial<double, 1>,
		   Polynomial<double, 2>,
		   Polynomial<double, 3 >> PolynomialDerivativeTypes;

TYPED_TEST_CASE(PolynomialTests, PolynomialTypes);
TYPED_TEST_CASE(PolynomialDerivativeTests, PolynomialDerivativeTypes);

TYPED_TEST(PolynomialTests, InitializationTests)
{
	EXPECT_NO_THROW(TypeParam poly);
	TypeParam poly;
	this->initializeConstructor(&poly);
	this->checkPolynomialConstructor(&poly);
};

TYPED_TEST(PolynomialTests, ArithmeticTests)
{
	TypeParam poly1;
	this->setPolynomialFromOffset(1, &poly1);
	TypeParam poly2;
	this->setPolynomialFromOffset(2, &poly2);
	this->checkPolynomialArithmetic(poly1, poly2);
};

TYPED_TEST(PolynomialDerivativeTests, DerivativeTests)
{
	TypeParam poly1;
	this->setPolynomialFromOffset(1, &poly1);
	this->checkPolynomialDerivative(poly1);
};

TYPED_TEST(PolynomialTests, NearZeroTests)
{
	TypeParam poly1;
	this->setPolynomialToSmallValue(&poly1);
	this->checkIsNearZero(&poly1);
};

TEST_F(PolynomialUtilityTests, UtilityTests)
{
	Polynomial<double, 0> p0_1(1.0);
	Polynomial<double, 0> p0_2(2.0);
	Polynomial<double, 1> p1_1(1.0, 2.0);
	Polynomial<double, 1> p1_2(2.0, 3.0);
	Polynomial<double, 2> p2_1(1.0, 2.0, 3.0);
	Polynomial<double, 2> p2_2(2.0, 3.0, 4.0);
	Polynomial<double, 3> p3_1(1.0, 2.0, 3.0, 4.0);
	Polynomial<double, 3> p3_2(2.0, 3.0, 4.0, 5.0);

	// degree n * degree m
	{
		auto result = p0_2 * p3_2;
		EXPECT_TRUE(result.isApprox(Polynomial<double, 3>(4.0, 6.0, 8.0, 10.0), epsilon));
	}

	// degree 1 * degree 1
	{
		auto result = p1_1 * p1_2;
		EXPECT_TRUE(result.isApprox(Polynomial<double, 2>(2.0, 7.0, 6.0), epsilon));
	}

	// degree 2 * degree 1
	{
		auto result = p2_1 * p1_2;
		EXPECT_TRUE(result.isApprox(Polynomial<double, 3>(2.0, 7.0, 12.0, 9.0), epsilon));
	}

	// degree 1 * degree 2
	{
		auto result = p1_1 * p2_2;
		EXPECT_TRUE(result.isApprox(Polynomial<double, 3>(2.0, 7.0, 10.0, 8.0), epsilon));
	}

	// degree 0^2
	{
		auto result_p0_1 = square(p0_1);
		EXPECT_TRUE(result_p0_1.isApprox(Polynomial<double, 0>(1.0), epsilon));
		auto result_p0_2 = square(p0_2);
		EXPECT_TRUE(result_p0_2.isApprox(Polynomial<double, 0>(4.0), epsilon));
	}

	// degree 1^2
	{
		auto result = square(p1_2);
		EXPECT_TRUE(result.isApprox(Polynomial<double, 2>(4.0, 12.0, 9.0), epsilon));
	}

	// Output
	{
		// Output
		std::ostringstream intervalOutput;
		intervalOutput << p3_1;
		EXPECT_EQ("(4*x^3 + 3*x^2 + 2*x + 1)", intervalOutput.str());
	}
};

}; // namespace Math
}; // namespace SurgSim
