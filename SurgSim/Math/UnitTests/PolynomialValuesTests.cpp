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

#include "SurgSim/Math/PolynomialValues.h"

namespace SurgSim
{

namespace Math
{

namespace
{
double epsilon = 1.0e-9;
}

TEST(PolynomialValuesTest, PolnomialDegree0Values)
{
	SurgSim::Math::Polynomial<double, 0> constantPoly(-24.0);
	SurgSim::Math::Interval<double> interval(-1.0, 10.0);
	PolynomialValues<double, 0> constantValues(constantPoly);
	EXPECT_DOUBLE_EQ(constantPoly.getCoefficient(0), constantValues.getPolynomial().getCoefficient(0));
	EXPECT_TRUE(Interval<double>(-24.0, -24.0).isApprox(constantValues.valuesOverInterval(interval), epsilon));
};

TEST(PolynomialValuesTest, PolnomialDegree1Values)
{
	SurgSim::Math::Polynomial<double, 1> linearPoly(-24.0, 2.0);
	SurgSim::Math::Interval<double> interval(-1.0, 10.0);
	PolynomialValues<double, 1> linearValues(linearPoly);
	EXPECT_DOUBLE_EQ(linearPoly.getCoefficient(0), linearValues.getPolynomial().getCoefficient(0));
	EXPECT_DOUBLE_EQ(linearPoly.getCoefficient(1), linearValues.getPolynomial().getCoefficient(1));
	EXPECT_TRUE(Interval<double>(-26.0, -4.0).isApprox(linearValues.valuesOverInterval(interval), epsilon));
};

TEST(PolynomialValuesTest, PolnomialDegree2Values)
{
	SurgSim::Math::Polynomial<double, 2> quadraticPoly(-8.0, 6.0, 2.0);
	SurgSim::Math::Interval<double> interval1(-1.0, 10.0);
	SurgSim::Math::Interval<double> interval2(-2.0, 10.0);
	PolynomialValues<double, 2> quadraticValues(quadraticPoly);
	EXPECT_DOUBLE_EQ(quadraticPoly.getCoefficient(0), quadraticValues.getPolynomial().getCoefficient(0));
	EXPECT_DOUBLE_EQ(quadraticPoly.getCoefficient(1), quadraticValues.getPolynomial().getCoefficient(1));
	EXPECT_DOUBLE_EQ(quadraticPoly.getCoefficient(2), quadraticValues.getPolynomial().getCoefficient(2));
	EXPECT_NEAR(6.0, quadraticValues.getDerivative().getCoefficient(0), epsilon);
	EXPECT_NEAR(4.0, quadraticValues.getDerivative().getCoefficient(1), epsilon);
	EXPECT_EQ(1, quadraticValues.getLocationsOfExtrema().getNumRoots());
	EXPECT_NEAR(-1.5, quadraticValues.getLocationsOfExtrema()[0], epsilon);
	EXPECT_TRUE(Interval<double>(-12.0, 252.0).isApprox(quadraticValues.valuesOverInterval(interval1), epsilon));
	EXPECT_TRUE(Interval<double>(-12.5, 252.0).isApprox(quadraticValues.valuesOverInterval(interval2), epsilon));
};

TEST(PolynomialValuesTest, ValuesOverIntervalTests)
{
	SurgSim::Math::Polynomial<double, 0> constantPoly(-24.0);
	SurgSim::Math::Polynomial<double, 1> linearPoly(-24.0, 2.0);
	SurgSim::Math::Polynomial<double, 2> quadraticPoly(-8.0, 6.0, 2.0);
	SurgSim::Math::Polynomial<double, 3> cubicPoly(7.0, -8.0, 6.0, 2.0);
	SurgSim::Math::Interval<double> interval(-1.0, 10.0);
	EXPECT_TRUE(Interval<double>(-24.0, -24.0).isApprox(valuesOverInterval(constantPoly, interval), epsilon));
	EXPECT_TRUE(Interval<double>(-26.0, -4.0).isApprox(valuesOverInterval(linearPoly, interval), epsilon));
	EXPECT_TRUE(Interval<double>(-12.0, 252.0).isApprox(valuesOverInterval(quadraticPoly, interval), epsilon));
};

}; // namespace Math
}; // namespace SurgSim
