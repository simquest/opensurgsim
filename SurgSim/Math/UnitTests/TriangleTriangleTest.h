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

#ifndef SURGSIM_MATH_UNITTESTS_TRIANGLETRIANGLETEST_H
#define SURGSIM_MATH_UNITTESTS_TRIANGLETRIANGLETEST_H


#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/UnitTests/Triangle.h"
#include "SurgSim/Math/Geometry.h"

namespace SurgSim
{

namespace Math
{

/// A base class for triangle-triangle tests, which has a preset list of test cases.
class TriangleTriangleTest
{
protected:
	typedef std::tuple<std::string,	// String to describe the scenario.
					   Triangle,	// The first triangle.
					   Triangle,	// The second triangle.
					   bool,		// Flag to indicate if the two triangles are expected to be found intersecting.
					   bool,		// Flag to indicate if expected contact info is available to check against.
					   Vector3d,	// Expected penetration point in the first triangle.
					   Vector3d>	// Expected penetration point in the second triangle.
		TriangleTriangleTestCase;

	/// A list of common test cases.
	std::vector<TriangleTriangleTestCase> m_testCases;

	/// Default constructor.
	TriangleTriangleTest()
	{
		double d = 5.0 * Geometry::DistanceEpsilon;
		Triangle t0(Vector3d(-5, 0, 0), Vector3d(0, 10, 0), Vector3d(5, 0, 0));
		Triangle t1;
		{
			std::string scenario = "vertex t1v0 inside t0v0";
			t1 = Triangle(t0.v0 + Vector3d(0, 0, d), t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v0, t0.v0));
		}
		{
			std::string scenario = "vertex t1v0 inside of triangle t0";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			t1 = Triangle(t1v0 + Vector3d(0, 0, d), t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v0, t1v0));
		}
		{
			std::string scenario = "vertex t1v0, t1v1 inside of triangle t0, at same depth";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			Vector3d t1v1 = t0.pointInTriangle(0.4, 0.4);
			t1 = Triangle(t1v0 + Vector3d(0, 0, d), t1v1 + Vector3d(0, 0, d), t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, false, t1.v1, t1v1));
		}
		{
			std::string scenario = "vertex t1v0, t1v1 inside of triangle t0, depth of t1v0 < t1v1";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			Vector3d t1v1 = t0.pointInTriangle(0.4, 0.4);
			t1 = Triangle(t1v0 + Vector3d(0, 0, d), t1v1 + Vector3d(0, 0, d * 2.0), t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v1, t1v1));
		}
		{
			std::string scenario = "vertex t1v0, t1v1 inside of triangle t0, depth of t1v0 > t1v1";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			Vector3d t1v1 = t0.pointInTriangle(0.4, 0.4);
			t1 = Triangle(t1v0 + Vector3d(0, 0, d * 2.0), t1v1 + Vector3d(0, 0, d), t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, true, true, t1.v0, t1v0));
		}
		{
			std::string scenario = "vertex t1v0 close to t0v0";
			t1 = Triangle(t0.v0 + t0.n, t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, false, false, t1.v0, t0.v0));
		}
		{
			std::string scenario = "vertex t1v0 close to the inside of triangle t0";
			Vector3d intersection = t0.pointInTriangle(0.2, 0.2);
			t1 = Triangle(intersection + t0.n , t0.v1 + t0.n * 2, t0.v2 + t0.n * 2);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, false, false, t1.v0, intersection));
		}
		{
			std::string scenario = "edge t1v0v1 through triangle t0";
			Vector3d t1v0 = t0.pointInTriangle(0.2, 0.2);
			t1 = Triangle(t1v0 + t0.n * 3, t0.v0 - t0.v0v2 * 4 + t0.n, t1v0 - t0.n * 4);
			Vector3d t0p;
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "Triangles parallel";
			t1 = Triangle(t0.v0 + t0.n * 3, t0.v1 + t0.n * 3, t0.v2 + t0.n * 3);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t1, t0, false, false, t1.v0, t1.v0));
		}

		Triangle T0(Vector3d(-5.0, 0, 0), Vector3d(5, 0, 0), Vector3d(0, 10, 0));
		T0.move(Vector3d(0, -3.333333333, 0));
		Triangle T1(Vector3d(-5.0, 0, 0), Vector3d(5, 0, 0), Vector3d(0, 10, 0));
		T1.move(Vector3d(0, -10, 0));
		T1.rotateByXDegrees(-90.0);

		{
			std::string scenario = "vertex t1v0 inside t0 - 1";
			Triangle t0(T0);
			Vector3d t0p(0, 0, 0);
			Triangle t1(T1);
			t1.move(Vector3d(0.0, 0.0, -0.1));
			Vector3d t1p(0, 0, -0.1);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 2";
			Triangle t0(T0);
			Vector3d t0p(0, -3.333333333, 0);
			Triangle t1(T1);
			t1.move(Vector3d(0.0, -3.3, -0.1));
			Vector3d t1p(0, -3.3, 0);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 3";
			Triangle t0(T0);
			Vector3d t0p(0, -3.22222222, 0);
			Triangle t1(T1);
			t1.move(Vector3d(0.0, -3.22222222, -0.1));
			Vector3d t1p(0, -3.22222222, -0.1);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 4";
			Triangle t0(T0);
			Vector3d t0p(0, 0, 0);
			Triangle t1(T1);
			t1.move(Vector3d(0.0, 0.0, -0.6));
			Vector3d t1p(0, 0, -0.6);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 5";
			Triangle t0(T0);
			Vector3d t0p(0, 0, 0);
			Triangle t1(T1);
			t1.rotateByZDegrees(180.0);
			t1.move(Vector3d(0.0, 0.0, -6.6));
			Vector3d t1p(0, 0, -6.6);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "vertex t1v0 inside t0 - 6";
			Triangle t0(T0);
			Vector3d t0p(t0.v2);
			Triangle t1(T1);
			t1.rotateByZDegrees(180.0);
			t1.move(Vector3d(0.0, 0.0, -6.7));
			Vector3d t1p(0, 0, 0);
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, true, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 1";
			Triangle t0(T0);
			Vector3d t0p;
			Triangle t1(T1);
			t1.move(Vector3d(-3.0, 0.0, -6.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 2";
			Triangle t0(T0);
			Vector3d t0p;
			Triangle t1(T1);
			t1.rotateByZDegrees(90.0);
			t1.move(Vector3d(0.0, -4.0, -5.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 3";
			Triangle t0(T0);
			Vector3d t0p;
			Triangle t1(T1);
			t1.rotateByYDegrees(180.0);
			t1.rotateByZDegrees(90.0);
			t1.move(Vector3d(0.0, -4.0, 6.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
		{
			std::string scenario = "edge (t1v0,t1v1) inside t0 - 4";
			Triangle t0(T0);
			Vector3d t0p;
			Triangle t1(T1);
			t1.rotateByYDegrees(180.0);
			t1.rotateByZDegrees(90.0);
			t1.move(Vector3d(4.0, -4.0, 6.0));
			Vector3d t1p;
			m_testCases.push_back(TriangleTriangleTestCase(scenario, t0, t1, true, false, t0p, t1p));
		}
	}
};


} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_UNITTESTS_TRIANGLETRIANGLETEST_H