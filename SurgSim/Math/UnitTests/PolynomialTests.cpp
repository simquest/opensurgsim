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

class PolynomialTests : public ::testing::Test
{
public:
	template <int order>
	void testConstructor()
	{
		switch (order)
		{
			case 0:
			{
				// Basic construction tests
				typedef Polynomial<0, double> PolynomialType;
				EXPECT_NO_THROW(PolynomialType poly);
				EXPECT_NO_THROW(PolynomialType poly(7.0));
				Polynomial<0, double> poly(1.0);
				checkPolynomialConstructor(&poly);
				break;
			}
			case 1:
			{
				typedef Polynomial<1, double> PolynomialType;
				EXPECT_NO_THROW(PolynomialType poly);
				EXPECT_NO_THROW(PolynomialType poly(7.0, 8.0));
				Polynomial<1, double> poly(1.0, 2.0);
				checkPolynomialConstructor(&poly);
				break;
			}
			case 2:
			{
				typedef Polynomial<2, double> PolynomialType;
				EXPECT_NO_THROW(PolynomialType poly);
				EXPECT_NO_THROW(PolynomialType poly(7.0, 8.0, 9.0));
				Polynomial<2, double> poly(1.0, 2.0, 3.0);
				checkPolynomialConstructor(&poly);
				break;
			}
			case 3:
			{
				typedef Polynomial<3, double> PolynomialType;
				EXPECT_NO_THROW(PolynomialType poly);
				EXPECT_NO_THROW(PolynomialType poly(7.0, 8.0, 9.0, 10.0));
				Polynomial<3, double> poly(1.0, 2.0, 3.0, 4.0);
				checkPolynomialConstructor(&poly);
				break;
			}
		}
	}

	template <int order>
	void testArithmetic()
	{
		switch (order)
		{
			case 0:
			{
				Polynomial<0, double> poly1(1.0);
				Polynomial<0, double> poly2(2.0);
				checkPolynomialArithmetic(poly1, poly2);
				break;
			}
			case 1:
			{
				Polynomial<1, double> poly1(1.0, 2.0);
				Polynomial<1, double> poly2(2.0, 3.0);
				checkPolynomialArithmetic(poly1, poly2);
				break;
			}
			case 2:
			{
				Polynomial<2, double> poly1(1.0, 2.0, 3.0);
				Polynomial<2, double> poly2(2.0, 3.0, 4.0);
				checkPolynomialArithmetic(poly1, poly2);
				break;
			}
			case 3:
			{
				Polynomial<3, double> poly1(1.0, 2.0, 3.0, 4.0);
				Polynomial<3, double> poly2(2.0, 3.0, 4.0, 5.0);
				checkPolynomialArithmetic(poly1, poly2);
				break;
			}
		}
	}

	template <int order>
	void testDerivative()
	{
		switch (order)
		{
			case 0:
			{
				Polynomial<0, double> poly1(1.0);
				checkPolynomialDerivative(poly1);
				break;
			}
			case 1:
			{
				Polynomial<1, double> poly1(1.0, 2.0);
				checkPolynomialDerivative(poly1);
				break;
			}
			case 2:
			{
				Polynomial<2, double> poly1(1.0, 2.0, 3.0);
				checkPolynomialDerivative(poly1);
				break;
			}
			case 3:
			{
				Polynomial<3, double> poly1(1.0, 2.0, 3.0, 4.0);
				checkPolynomialDerivative(poly1);
				break;
			}
		}
	}

	template <int order>
	void testNearZero()
	{
		switch (order)
		{
			case 0:
			{
				Polynomial<0, double> poly1(epsilon / 2.0);
				checkIsNearZero(&poly1);
				break;
			}
			case 1:
			{
				Polynomial<1, double> poly1(epsilon / 2.0, epsilon / 2.0);
				checkIsNearZero(&poly1);
				break;
			}
			case 2:
			{
				Polynomial<2, double> poly1(epsilon / 2.0, epsilon / 2.0, epsilon / 2.0);
				checkIsNearZero(&poly1);
				break;
			}
			case 3:
			{
				Polynomial<3, double> poly1(epsilon / 2.0, epsilon / 2.0, epsilon / 2.0, epsilon / 2.0);
				checkIsNearZero(&poly1);
				break;
			}
		}
	}

	template <int order>
	void checkPolynomialConstructor(Polynomial<order, double>* poly)
	{
		// Check get ... up to order
		for (size_t counter = 0; counter <= order; ++counter)
		{
			EXPECT_EQ(counter + 1.0, poly->getCoefficient(counter));
		}

		// Check get ... beyond order
		for (size_t counter = order + 1; counter < 5; ++counter)
		{
			EXPECT_EQ(0.0, poly->getCoefficient(counter));
		}

		// Check set up to order
		for (size_t counter = 0; counter <= order; ++counter)
		{
			poly->setCoefficient(counter, static_cast<double>(counter));
			EXPECT_EQ(counter, poly->getCoefficient(counter));
		}

		// Check set beyond order
		EXPECT_THROW(poly->setCoefficient(order + 1, 12.0), SurgSim::Framework::AssertionFailure);
	}

	template <int order>
	void checkPolynomialArithmetic(const Polynomial<order, double>& poly1,
								   const Polynomial<order, double>& poly2)
	{
		Polynomial<order, double> scratch;

		// Check evaluate ...
		double evaluate = static_cast<double>(order + 1);
		for (int counter = order - 1; counter >= 0; --counter)
		{
			evaluate = (0.5 * evaluate) + counter + 1;
		}
		EXPECT_NEAR(evaluate, poly1.evaluate(0.5), epsilon);

		// Check negation ...
		scratch = -poly1;
		for (size_t counter = 0; counter <= order; ++counter)
		{
			EXPECT_EQ(-1 * (static_cast<double>(counter) + 1.0), scratch.getCoefficient(counter));
		}

		// Check add ...
		scratch = poly1 + poly2;
		for (size_t counter = 0; counter <= order; ++counter)
		{
			EXPECT_EQ(2 * static_cast<double>(counter) + 3.0, scratch.getCoefficient(counter));
		}

		scratch = poly1;
		scratch += poly2;
		for (size_t counter = 0; counter <= order; ++counter)
		{
			EXPECT_EQ(2 * static_cast<double>(counter) + 3.0, scratch.getCoefficient(counter));
		}

		// Check subtract ...
		scratch = poly1 - poly2;
		for (size_t counter = 0; counter <= order; ++counter)
		{
			EXPECT_EQ(-1.0, scratch.getCoefficient(counter));
		}

		scratch = poly1;
		scratch -= poly2;
		for (size_t counter = 0; counter <= order; ++counter)
		{
			EXPECT_EQ(-1.0, scratch.getCoefficient(counter));
		}
	}

	template <int order>
	void checkPolynomialDerivative(const Polynomial<order, double>& poly1)
	{
		// Check derivative ...
		auto smallScratch = poly1.derivative();
		for (size_t counter = 0; counter < order; ++counter)
		{
			EXPECT_EQ((counter + 2.0) * (counter + 1.0), smallScratch.getCoefficient(counter));
		}
		EXPECT_EQ(0.0, smallScratch.getCoefficient(order));
	}

	template <int order>
	void checkIsNearZero(Polynomial<order, double>* poly1)
	{
		EXPECT_TRUE(poly1->isNearZero(epsilon));
		for (size_t counter = 0; counter <= order; counter++)
		{
			poly1->setCoefficient(counter, 2 * epsilon);
			EXPECT_FALSE(poly1->isNearZero(epsilon));
			poly1->setCoefficient(counter, epsilon / 2.0);
		}
	}
};

TEST_F(PolynomialTests, InitializationTests)
{
	testConstructor<0>();
	testConstructor<1>();
	testConstructor<2>();
	testConstructor<3>();
};

TEST_F(PolynomialTests, ArithmeticTests)
{
	testArithmetic<0>();
	testArithmetic<1>();
	testArithmetic<2>();
	testArithmetic<3>();
};

TEST_F(PolynomialTests, DerivativeTests)
{
	testDerivative<0>();
	testDerivative<1>();
	testDerivative<2>();
	testDerivative<3>();
};

TEST_F(PolynomialTests, NearZeroTests)
{
	testNearZero<0>();
	testNearZero<1>();
	testNearZero<2>();
	testNearZero<3>();
};

TEST_F(PolynomialTests, UtilityTests)
{
	Polynomial<0, double> p0_1(1.0);
	Polynomial<0, double> p0_2(2.0);
	Polynomial<1, double> p1_1(1.0, 2.0);
	Polynomial<1, double> p1_2(2.0, 3.0);
	Polynomial<2, double> p2_1(1.0, 2.0, 3.0);
	Polynomial<2, double> p2_2(2.0, 3.0, 4.0);
	Polynomial<3, double> p3_1(1.0, 2.0, 3.0, 4.0);
	Polynomial<3, double> p3_2(2.0, 3.0, 4.0, 5.0);

	// order n * order m
	{
		auto result = p0_2 * p3_2;
		EXPECT_EQ(4.0, result.getCoefficient(0));
		EXPECT_EQ(6.0, result.getCoefficient(1));
		EXPECT_EQ(8.0, result.getCoefficient(2));
		EXPECT_EQ(10.0, result.getCoefficient(3));
	}

	// order 1 * order 1
	{
		auto result = p1_1 * p1_2;
		EXPECT_EQ(2.0, result.getCoefficient(0));
		EXPECT_EQ(7.0, result.getCoefficient(1));
		EXPECT_EQ(6.0, result.getCoefficient(2));
	}

	// order 2 * order 1
	{
		auto result = p2_1 * p1_2;
		EXPECT_EQ(2.0, result.getCoefficient(0));
		EXPECT_EQ(7.0, result.getCoefficient(1));
		EXPECT_EQ(12.0, result.getCoefficient(2));
		EXPECT_EQ(9.0, result.getCoefficient(3));
	}

	// order 1 * order 2
	{
		auto result = p1_1 * p2_2;
		EXPECT_EQ(2.0, result.getCoefficient(0));
		EXPECT_EQ(7.0, result.getCoefficient(1));
		EXPECT_EQ(10.0, result.getCoefficient(2));
		EXPECT_EQ(8.0, result.getCoefficient(3));
	}

	// order 0^2
	{
		auto result_p0_1 = square(p0_1);
		EXPECT_EQ(1.0, result_p0_1.getCoefficient(0));
		auto result_p0_2 = square(p0_2);
		EXPECT_EQ(4.0, result_p0_2.getCoefficient(0));
	}

	// order 1^2
	{
		auto result = square(p1_2);
		EXPECT_EQ(4.0, result.getCoefficient(0));
		EXPECT_EQ(12.0, result.getCoefficient(1));
		EXPECT_EQ(9.0, result.getCoefficient(2));
	}

	// Output
	{
		// Output
		std::ostringstream intervalOutput;
		intervalOutput << p3_1;
		EXPECT_EQ("(4*x^3 + 3*x^2 + 2*x + 1*1)", intervalOutput.str());
	}
};

}; // namespace Math
}; // namespace SurgSim
