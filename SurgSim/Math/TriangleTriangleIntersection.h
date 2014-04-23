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

#ifndef SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_H
#define SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_H


#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{

/// Check if the two triangles intersect using separating axis test.
/// Algorithm is implemented from http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tritri.pdf
///
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t1v0,t1v1,t1v2 Vertices of the first triangle.
/// \param t2v0,t2v1,t2v2 Vertices of the second triangle.
/// \param t1n Normal of the first triangle.
/// \param t2n Normal of the second triangle
/// \return True, if intersection is detected.
template <class T, int MOpt> inline
bool checkTriangleTriangleIntersection(
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1n,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2n)
{
	// Epsilon used in this function.
	T e = T(Geometry::DistanceEpsilon);

	// struct to store a plane.
	struct Plane
	{
	public:
		Plane(const Eigen::Matrix<T, 3, 1, MOpt>& n, T d)
			: m_n(n), m_d(d)
		{}
		Eigen::Matrix<T, 3, 1, MOpt> m_n;
		T m_d;
	};

	// Variable names mentioned here are the notations used in the paper:
	// T1		- Triangle with vertices (t1v0, t1v1, t1v2).
	// T2		- Triangle with vertices (t2v0, t2v1, t2v2).
	// d1[3]	- Signed distances of the vertices of T1 from the plane of T2.
	// d2[3]	- Signed distances of the vertices of T2 from the plane of T1.
	// D		- Separating axis used for the test. This is calculated as the cross products of the triangle normals.
	// pv1[3]	- Projection of the vertices of T1 onto the separating axis (D).
	// pv2[3]	- Projection of the vertices of T2 onto the separating axis (D).
	// t[2]		- The intersection between T1 and D is a line segment.
	//			  t[0] and t[1] are the parametric representation of the ends of this line segment.
	// s[2]		- The intersection between T2 and D is a line segment.
	//			  s[0] and s[1] are the parametric representation of the ends of this line segment.

	// Some extra variables:
	// p1		- The plane of the triangle T1.
	// p2		- The plane of the triangle T2.

	// Early Rejection test:
	// If all the vertices of one triangle are on one side of the plane of the other triangle,
	// there is no intersection.

	// Check if all the vertices of T2 are on one side of p1.
	Plane p1(t1n, -t1v0.dot(t1n));
	T d2[3] = {t2v0.dot(p1.m_n) + p1.m_d, t2v1.dot(p1.m_n) + p1.m_d, t2v2.dot(p1.m_n) + p1.m_d};
	if ((d2[0] <= e && d2[1] <= e && d2[2] <= e) ||
		(d2[0] >= -e && d2[1] >= -e && d2[2] >= -e))
	{
		return false;
	}

	// Check if all the vertices of T1 are on one side of p2.
	Plane p2(t2n, -t2v0.dot(t2n));
	T d1[3] = {t1v0.dot(p2.m_n) + p2.m_d, t1v1.dot(p2.m_n) + p2.m_d, t1v2.dot(p2.m_n) + p2.m_d};
	if ((d1[0] <= e && d1[1] <= e && d1[2] <= e) ||
		(d1[0] >= -e && d1[1] >= -e && d1[2] >= -e))
	{
		return false;
	}

	// The separating axis.
	Eigen::Matrix<T, 3, 1, MOpt> D = p1.m_n.cross(p2.m_n);

	// Projection of the triangle vertices on the separating axis.
	T pv1[3] = {D.dot(t1v0), D.dot(t1v1), D.dot(t1v2)};
	T pv2[3] = {D.dot(t2v0), D.dot(t2v1), D.dot(t2v2)};

	T t[3], s[3];
	int tCount = 0, sCount = 0;

	// Loop through the edges of each triangle and "clip" the edges to be on the plane of
	// the other triangle.
	for (int i = 0; i < 3; ++i)
	{
		int j = (i + 1) % 3;

		// First triangle.
		if (d1[i] < -e && d1[j] >= -e)
		{
			// i is under p2. j is on or above p2.
			if (d1[j] < e)
			{
				// j is on p2. No clipping is necessary.
				t[tCount++] = pv1[j];
			}
			else
			{
				// j is above p2. Clip it to be on p2.
				t[tCount++] = pv1[i] + (pv1[j] - pv1[i]) * (d1[i] / (d1[i] - d1[j]));
			}
		}
		else if (d1[i] > e && d1[j] <= e)
		{
			// i is above p2. j is on or under p2.
			if (d1[j] > -e)
			{
				// j is on p2. No clipping is necessary.
				t[tCount++] = pv1[j];
			}
			else
			{
				// j is under p2. Clip it to be on p2.
				t[tCount++] = pv1[i] + (pv1[j] - pv1[i]) * (d1[i] / (d1[i] - d1[j]));
			}
		}

		// Second triangle.
		if (d2[i] < -e && d2[j] >= -e)
		{
			if (d2[j] < e)
			{
				s[sCount++] = pv2[j];
			}
			else
			{
				s[sCount++] = pv2[i] + (pv2[j] - pv2[i]) * (d2[i] / (d2[i] - d2[j]));
			}
		}
		else if (d2[i] > e && d2[j] <= e)
		{
			if (d2[j] > -e)
			{
				s[sCount++] = pv2[j];
			}
			else
			{
				s[sCount++] = pv2[i] + (pv2[j] - pv2[i]) * (d2[i] / (d2[i] - d2[j]));
			}
		}
	}

	SURGSIM_ASSERT(tCount == 2 && sCount == 2)
		<< "The intersection between the triangle and the separating axis is not a line segment."
		<< " This scenario cannot happen, at this point in the process.";

	// Check if (t[0], t[1]) and (s[0], s[1]) overlap.
	return !(t[0] <= s[0] && t[0] <= s[1] && t[1] <= s[0] && t[1] <= s[1]) &&
		   !(t[0] >= s[0] && t[0] >= s[1] && t[1] >= s[0] && t[1] >= s[1]);
}

/// Check if the two triangles intersect using separating axis test.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t1v0,t1v1,t1v2 Vertices of the first triangle.
/// \param t2v0,t2v1,t2v2 Vertices of the second triangle.
/// \return True, if intersection is detected.
template <class T, int MOpt> inline
bool checkTriangleTriangleIntersection(
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t2v2)
{
	Eigen::Matrix<T, 3, 1, MOpt> t1n = (t1v1 - t1v0).cross(t1v2 - t1v0);
	Eigen::Matrix<T, 3, 1, MOpt> t2n = (t2v1 - t2v0).cross(t2v2 - t2v0);
	if (t1n.isZero() || t2n.isZero())
	{
		// Degenerate triangle(s) passed to checkTriangleTriangleIntersection.
		return false;
	}
	t1n.normalize();
	t2n.normalize();
	return checkTriangleTriangleIntersection(t1v0, t1v1, t1v2, t2v0, t2v1, t2v2, t1n, t2n);
}


} // namespace Math

} // namespace SurgSim


#endif // SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_H