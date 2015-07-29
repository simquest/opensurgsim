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

TEST_F(PolynomialRootsTest, PolynomialDegree1Roots)
{
	// Degenerate Linear, infinite roots
	SurgSim::Math::Polynomial<double, 1> degenerate(epsilon / 2.0, epsilon / 2.0);
	PolynomialRoots<double, 1> degenerateRoots(degenerate, 1.0e-09);
	EXPECT_TRUE(degenerateRoots.isDegenerate());
	EXPECT_EQ(-1, degenerateRoots.getNumRoots());
	EXPECT_THROW(degenerateRoots[0], SurgSim::Framework::AssertionFailure);

	// Semi-degenerate Linear, no roots
	SurgSim::Math::Polynomial<double, 1> constantPoly(2.0, epsilon / 2.0);
	PolynomialRoots<double, 1>constantPolyRoots(constantPoly, 1.0e-09);
	EXPECT_FALSE(constantPolyRoots.isDegenerate());
	EXPECT_EQ(0, constantPolyRoots.getNumRoots());
	EXPECT_THROW(constantPolyRoots[0], SurgSim::Framework::AssertionFailure);

	// Linear, one root
	SurgSim::Math::Polynomial<double, 1> linearPoly(-24.0, 2.0);
	PolynomialRoots<double, 1>linearPolyRoots(linearPoly, 1.0e-09);
	EXPECT_FALSE(linearPolyRoots.isDegenerate());
	EXPECT_EQ(1, linearPolyRoots.getNumRoots());
	EXPECT_NEAR(12.0, linearPolyRoots[0], epsilon);
	EXPECT_THROW(linearPolyRoots[1], SurgSim::Framework::AssertionFailure);
};

TEST_F(PolynomialRootsTest, PolynomialDegree2Roots)
{
	// Degenerate Linear, infinite roots
	SurgSim::Math::Polynomial<double, 2> degenerate(epsilon / 2.0, epsilon / 2.0, epsilon / 2.0);
	PolynomialRoots<double, 2>degenerateRoots(degenerate, 1.0e-09);
	EXPECT_TRUE(degenerateRoots.isDegenerate());
	EXPECT_EQ(-1, degenerateRoots.getNumRoots());
	EXPECT_THROW(degenerateRoots[0], SurgSim::Framework::AssertionFailure);

	// Semi-degenerate Linear, no roots
	SurgSim::Math::Polynomial<double, 2> constantPoly(2.0, epsilon / 2.0, epsilon / 2.0);
	PolynomialRoots<double, 2>constantPolyRoots(constantPoly, 1.0e-09);
	EXPECT_FALSE(constantPolyRoots.isDegenerate());
	EXPECT_EQ(0, constantPolyRoots.getNumRoots());
	EXPECT_THROW(constantPolyRoots[0], SurgSim::Framework::AssertionFailure);

	// Linear, one root
	SurgSim::Math::Polynomial<double, 2> linearPoly(-24.0, 2.0, epsilon / 2.0);
	PolynomialRoots<double, 2>linearPolyRoots(linearPoly, 1.0e-09);
	EXPECT_FALSE(linearPolyRoots.isDegenerate());
	EXPECT_EQ(1, linearPolyRoots.getNumRoots());
	EXPECT_NEAR(12.0, linearPolyRoots[0], epsilon);
	EXPECT_THROW(linearPolyRoots[1], SurgSim::Framework::AssertionFailure);

	// Quadratic, two imaginary roots
	SurgSim::Math::Polynomial<double, 2> imaginaryPoly(1.0, epsilon / 8.0, 1.0);
	PolynomialRoots<double, 2>imaginaryPolyRoots(imaginaryPoly, 1.0e-09);
	EXPECT_FALSE(imaginaryPolyRoots.isDegenerate());
	EXPECT_EQ(0, imaginaryPolyRoots.getNumRoots());
	EXPECT_THROW(imaginaryPolyRoots[0], SurgSim::Framework::AssertionFailure);

	// Quadratic, one (duplicate) root
	SurgSim::Math::Polynomial<double, 2> oneRootPoly(288.0, -48.0, 2.0);
	PolynomialRoots<double, 2>oneRootPolyRoots(oneRootPoly, 1.0e-09);
	EXPECT_FALSE(oneRootPolyRoots.isDegenerate());
	EXPECT_EQ(1, oneRootPolyRoots.getNumRoots());
	EXPECT_NEAR(12.0, oneRootPolyRoots[0], epsilon);
	EXPECT_THROW(oneRootPolyRoots[1], SurgSim::Framework::AssertionFailure);

	// Quadratic
	SurgSim::Math::Polynomial<double, 2> quadraticPoly(-8.0, 6.0, 2.0);
	PolynomialRoots<double, 2>quadraticPolyRoots(quadraticPoly, 1.0e-09);
	EXPECT_FALSE(quadraticPolyRoots.isDegenerate());
	EXPECT_EQ(2, quadraticPolyRoots.getNumRoots());
	EXPECT_NEAR(-4.0, quadraticPolyRoots[0], epsilon);
	EXPECT_NEAR(1.0, quadraticPolyRoots[1], epsilon);
	EXPECT_THROW(quadraticPolyRoots[2], SurgSim::Framework::AssertionFailure);
};

}; // namespace Math
}; // namespace SurgSim
