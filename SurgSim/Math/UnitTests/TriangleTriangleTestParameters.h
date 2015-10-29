// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License";
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

#ifndef SURGSIM_MATH_UNITTESTS_TRIANGLETRIANGLETESTPARAMETERS_H
#define SURGSIM_MATH_UNITTESTS_TRIANGLETRIANGLETESTPARAMETERS_H


#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/UnitTests/MockTriangle.h"
#include "SurgSim/Math/Geometry.h"

namespace SurgSim
{

namespace Math
{

/// A base class for triangle-triangle tests, which has a preset list of test cases.
class TriangleTriangleTestParameters
{
protected:
	typedef std::tuple<std::string,	// String to describe the scenario.
					   MockTriangle,	// The first triangle.
					   MockTriangle,	// The second triangle.
					   bool,		// Flag to indicate if the two triangles are expected to be found intersecting.
					   bool,		// Flag to indicate if expected contact info is available to check against.
					   Vector3d,	// Expected penetration point in the first triangle.
					   Vector3d>	// Expected penetration point in the second triangle.
			TriangleTriangleTestCase;

	/// A list of common test cases.
	std::vector<TriangleTriangleTestCase> m_testCases;

	/// Default constructor.
	TriangleTriangleTestParameters()
	{
		double d = 5.0 * Geometry::DistanceEpsilon;
		MockTriangle t0(Vector3d(-5, 0, 0), Vector3d(0, 10, 0), Vector3d(5, 0, 0));
		MockTriangle t1;
		{
			std::string scenario = "vertex t1v0 inside t0v0";
			t1 = MockTriangle(t0.v0 + Vector3d(0, 0, d), t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v0, t0.v0));
		}
		{
			std::string scenario = "vertex t1v0 inside of triangle t0";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			t1 = MockTriangle(t1v0 + Vector3d(0, 0, d), t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v0, t1v0));
		}
		{
			std::string scenario = "vertex t1v0, t1v1 inside of triangle t0, at same depth";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			Vector3d t1v1 = t0.pointInTriangle(0.4, 0.4);
			t1 = MockTriangle(t1v0 + Vector3d(0, 0, d), t1v1 + Vector3d(0, 0, d), t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, false, t1.v1, t1v1));
		}
		{
			std::string scenario = "vertex t1v0, t1v1 inside of triangle t0, depth of t1v0 < t1v1";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			Vector3d t1v1 = t0.pointInTriangle(0.4, 0.4);
			t1 = MockTriangle(t1v0 + Vector3d(0, 0, d), t1v1 + Vector3d(0, 0, d * 2.0), t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v1, t1v1));
		}
		{
			std::string scenario = "vertex t1v0, t1v1 inside of triangle t0, depth of t1v0 > t1v1";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			Vector3d t1v1 = t0.pointInTriangle(0.4, 0.4);
			t1 = MockTriangle(t1v0 + Vector3d(0, 0, d * 2.0), t1v1 + Vector3d(0, 0, d), t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v0, t1v0));
		}
		{
			std::string scenario = "vertex t1v0 close to t0v0";
			t1 = MockTriangle(t0.v0 + t0.n, t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, false, false, t1.v0, t0.v0));
		}
		{
			std::string scenario = "vertex t1v0 close to the inside of triangle t0";
			Vector3d intersection = t0.pointInTriangle(0.2, 0.2);
			t1 = MockTriangle(intersection + t0.n , t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, false, false, t1.v0, intersection));
		}
		{
			std::string scenario = "edge t1v0v1 through triangle t0";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			t1 = MockTriangle(t1v0 + t0.n * 3, t0.v0 - t0.v0v2 * 4 + t0.n, t1v0 - t0.n * 4);
			Vector3d t0p;
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "Triangles parallel";
			t1 = MockTriangle(t0.v0 + t0.n * 3, t0.v1 + t0.n * 3, t0.v2 + t0.n * 3);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, false, false, t1.v0, t1.v0));
		}

		MockTriangle T0(Vector3d(-5.0, 0, 0), Vector3d(5, 0, 0), Vector3d(0, 10, 0));
		T0.translate(Vector3d(0, -3.333333333, 0));
		MockTriangle T1(Vector3d(-5.0, 0, 0), Vector3d(5, 0, 0), Vector3d(0, 10, 0));
		T1.translate(Vector3d(0, -10, 0));
		T1.rotateAboutXBy(-90.0);

		{
			std::string scenario = "vertex t1v0 inside t0 - 1";
			MockTriangle t0(T0);
			Vector3d t0p(0, 0, 0);
			MockTriangle t1(T1);
			t1.translate(Vector3d(0.0, 0.0, -0.1));
			Vector3d t1p(0, 0, -0.1);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 2";
			MockTriangle t0(T0);
			Vector3d t0p;
			MockTriangle t1(T1);
			t1.translate(Vector3d(0.0, -3.3, -0.1));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 3";
			MockTriangle t0(T0);
			Vector3d t0p(0, -3.22222222, 0);
			MockTriangle t1(T1);
			t1.translate(Vector3d(0.0, -3.22222222, -0.1));
			Vector3d t1p(0, -3.22222222, -0.1);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 4";
			MockTriangle t0(T0);
			Vector3d t0p(0, 0, 0);
			MockTriangle t1(T1);
			t1.translate(Vector3d(0.0, 0.0, -0.6));
			Vector3d t1p(0, 0, -0.6);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 5";
			MockTriangle t0(T0);
			Vector3d t0p(0, 0, 0);
			MockTriangle t1(T1);
			t1.rotateAboutZBy(180.0);
			t1.translate(Vector3d(0.0, 0.0, -6.6));
			Vector3d t1p(0, 0, -6.6);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 6";
			MockTriangle t0(T0);
			Vector3d t0p(t0.v2);
			MockTriangle t1(T1);
			t1.rotateAboutZBy(180.0);
			t1.translate(Vector3d(0.0, 0.0, -6.7));
			Vector3d t1p(0, 0, 0);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 1";
			MockTriangle t0(T0);
			Vector3d t0p;
			MockTriangle t1(T1);
			t1.translate(Vector3d(-3.0, 0.0, -6.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 2";
			MockTriangle t0(T0);
			Vector3d t0p;
			MockTriangle t1(T1);
			t1.rotateAboutZBy(90.0);
			t1.translate(Vector3d(0.0, -3.5, -9.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 3";
			MockTriangle t0(T0);
			Vector3d t0p;
			MockTriangle t1(T1);
			t1.rotateAboutYBy(180.0);
			t1.rotateAboutZBy(90.0);
			t1.translate(Vector3d(0.0, -4.0, 6.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 4";
			MockTriangle t0(T0);
			Vector3d t0p;
			MockTriangle t1(T1);
			t1.rotateAboutYBy(180.0);
			t1.rotateAboutZBy(90.0);
			t1.translate(Vector3d(4.0, -4.0, 6.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "Failed case in Stapler demo - 1";
			MockTriangle t0(Vector3d(-0.0063320380397585046, 0.0028276973112210521, 0.014661107730129588),
						Vector3d(-0.012108385700376603, 0.0012180224983028599, 0.011926511653863735),
						Vector3d(-0.016994324947197881, -0.011073183474260971, 0.022191024814086323));
			Vector3d t0p;
			MockTriangle t1(Vector3d(-0.031071999999999999, 0.0028570000000000002, 0.012547000000000001),
						Vector3d(-0.0016770000000000001, -0.0010070000000000001, 0.0048640000000000003),
						Vector3d(-0.0013829999999999999, -0.0010030000000000000, 0.012973000000000000));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "Failed case in Stapler demo - 2";
			MockTriangle t0(Vector3d(3.4602053093157404, -1.1441614263267368, 37.870346680755349),
						Vector3d(3.3720821269094003, -0.20927613787449697, 118.95490947665477),
						Vector3d(3.2033802246727975, -36.106495941162471, 119.36861234419522));
			Vector3d t0p;
			MockTriangle t1(Vector3d(0.50000000000000002, 3.8999999999999999, 53.099999999999996),
						Vector3d(0.50000000000000002, -3.8999999999999999, 53.099999999999996),
						Vector3d(21.299999999999999, 0.00000000000000000, 53.299999999999997));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "Failed case in Vascular";
			MockTriangle t0(Vector3d(0.12419200000000000, -0.0027260000000000001, -0.039763000000000000),
				Vector3d(0.10276100000000001, -0.0020000000000000000, -0.034617000000000002),
				Vector3d(0.10060200000000000, -0.0026749999999999999, -0.036646999999999999));
			Vector3d t0p;
			MockTriangle t1(Vector3d(0.13072899974405072, -0.0023620000000000000, -0.033049000000000002),
				Vector3d(0.12155499974405069, -0.0017220000000000000, -0.039350000000000003),
				Vector3d(0.12419199974405069, -0.0027260000000000001, -0.039763000000000000));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, false, false, t0p, t1p));
		}
	}
};


} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_UNITTESTS_TRIANGLETRIANGLETESTPARAMETERS_H