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

#include "SurgSim/Math/PolynomialRoots.h"

namespace SurgSim
{
namespace Math
{

namespace
{
double epsilon = 1.0e-9;
}

class PolynomialRootsTest : public ::testing::Test
{
};

TEST_F(PolynomialRootsTest, PolnomialDegree1Roots)
{
	// Degenerate Linear, infinite roots
	SurgSim::Math::Polynomial<1, double> degenerate(epsilon / 2.0, epsilon / 2.0);
	PolynomialRoots<1, double>degenerateRoots(degenerate, 1.0e-09);
	EXPECT_TRUE(degenerateRoots.isDegenerate());
	EXPECT_EQ(-1, degenerateRoots.getNumRoots());
	EXPECT_THROW(degenerateRoots[0], SurgSim::Framework::AssertionFailure);

	// Semi-degenerate Linear, no roots
	SurgSim::Math::Polynomial<1, double> constantPoly(2.0, epsilon / 2.0);
	PolynomialRoots<1, double>constantPolyRoots(constantPoly, 1.0e-09);
	EXPECT_FALSE(constantPolyRoots.isDegenerate());
	EXPECT_EQ(0, constantPolyRoots.getNumRoots());
	EXPECT_THROW(constantPolyRoots[0], SurgSim::Framework::AssertionFailure);

	// Linear, one root
	SurgSim::Math::Polynomial<1, double> linearPoly(-24.0, 2.0);
	PolynomialRoots<1, double>linearPolyRoots(linearPoly, 1.0e-09);
	EXPECT_FALSE(linearPolyRoots.isDegenerate());
	EXPECT_EQ(1, linearPolyRoots.getNumRoots());
	EXPECT_NEAR(12.0, linearPolyRoots[0], epsilon);
	EXPECT_THROW(linearPolyRoots[1], SurgSim::Framework::AssertionFailure);
};

TEST_F(PolynomialRootsTest, PolnomialDegree2Roots)
{
	// Degenerate Linear, infinite roots
	SurgSim::Math::Polynomial<2, double> degenerate(epsilon / 2.0, epsilon / 2.0, epsilon / 2.0);
	PolynomialRoots<2, double>degenerateRoots(degenerate, 1.0e-09);
	EXPECT_TRUE(degenerateRoots.isDegenerate());
	EXPECT_EQ(-1, degenerateRoots.getNumRoots());
	EXPECT_THROW(degenerateRoots[0], SurgSim::Framework::AssertionFailure);

	// Semi-degenerate Linear, no roots
	SurgSim::Math::Polynomial<2, double> constantPoly(2.0, epsilon / 2.0, epsilon / 2.0);
	PolynomialRoots<2, double>constantPolyRoots(constantPoly, 1.0e-09);
	EXPECT_FALSE(constantPolyRoots.isDegenerate());
	EXPECT_EQ(0, constantPolyRoots.getNumRoots());
	EXPECT_THROW(constantPolyRoots[0], SurgSim::Framework::AssertionFailure);

	// Linear, one root
	SurgSim::Math::Polynomial<2, double> linearPoly(-24.0, 2.0, epsilon / 2.0);
	PolynomialRoots<2, double>linearPolyRoots(linearPoly, 1.0e-09);
	EXPECT_FALSE(linearPolyRoots.isDegenerate());
	EXPECT_EQ(1, linearPolyRoots.getNumRoots());
	EXPECT_NEAR(12.0, linearPolyRoots[0], epsilon);
	EXPECT_THROW(linearPolyRoots[1], SurgSim::Framework::AssertionFailure);

	// Quadratic, two imaginary roots
	SurgSim::Math::Polynomial<2, double> imaginaryPoly(1.0, epsilon / 8.0, 1.0);
	PolynomialRoots<2, double>imaginaryPolyRoots(imaginaryPoly, 1.0e-09);
	EXPECT_FALSE(imaginaryPolyRoots.isDegenerate());
	EXPECT_EQ(0, imaginaryPolyRoots.getNumRoots());
	EXPECT_THROW(imaginaryPolyRoots[0], SurgSim::Framework::AssertionFailure);

	// Quadratic, one (duplicate) root
	SurgSim::Math::Polynomial<2, double> oneRootPoly(288.0, -48.0, 2.0);
	PolynomialRoots<2, double>oneRootPolyRoots(oneRootPoly, 1.0e-09);
	EXPECT_FALSE(oneRootPolyRoots.isDegenerate());
	EXPECT_EQ(1, oneRootPolyRoots.getNumRoots());
	EXPECT_NEAR(12.0, oneRootPolyRoots[0], epsilon);
	EXPECT_THROW(oneRootPolyRoots[1], SurgSim::Framework::AssertionFailure);

	// Quadratic
	SurgSim::Math::Polynomial<2, double> quadraticPoly(-8.0, 6.0, 2.0);
	PolynomialRoots<2, double>quadraticPolyRoots(quadraticPoly, 1.0e-09);
	EXPECT_FALSE(quadraticPolyRoots.isDegenerate());
	EXPECT_EQ(2, quadraticPolyRoots.getNumRoots());
	EXPECT_NEAR(-4.0, quadraticPolyRoots[0], epsilon);
	EXPECT_NEAR(1.0, quadraticPolyRoots[1], epsilon);
	EXPECT_THROW(quadraticPolyRoots[2], SurgSim::Framework::AssertionFailure);
};

TEST_F(PolynomialRootsTest, SolveAssertionTests)
{
	std::array<double, 0> zeroArray;
	std::array<double, 1> oneArray;
	int numRoots;

	// Linear case, 0 length array
	EXPECT_THROW((solve<0, double>(5.0, 5.0, epsilon, &numRoots, &zeroArray)),
				 SurgSim::Framework::AssertionFailure);

	// Quadratic, degenerate to linear.
	EXPECT_THROW((solve<0, double>(0.0, 5.0, 5.0, epsilon, &numRoots, &zeroArray)),
				 SurgSim::Framework::AssertionFailure);

	// Quadratic, duplicate root.
	EXPECT_THROW((solve<0, double>(1.0, -24.0, 144.0, epsilon, &numRoots, &zeroArray)),
				 SurgSim::Framework::AssertionFailure);

	// Quadratic, two roots.
	EXPECT_THROW((solve<1, double>(2.0, 6.0, -8.0, epsilon, &numRoots, &oneArray)),
				 SurgSim::Framework::AssertionFailure);
};

}; // namespace Math
}; // namespace SurgSim
