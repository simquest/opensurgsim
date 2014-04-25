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

#ifndef SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_INL_H
#define SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_INL_H


namespace SurgSim
{

namespace Math
{


/// A local class to represent a plane.
template <class T, int MOpt>
class Plane
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;

public:
	/// Contructor takes in th eplane normal and a point on the plane.
	/// \param normal Normal of the plane.
	/// \param pointOnPlane A point on the plane.
	Plane(const Vector3& normal, const Vector3& pointOnPlane)
		: m_normal(normal), m_d(-pointOnPlane.dot(normal))
	{}

	/// Calculate the signed distance of the given point from the plane.
	/// \param point Point whose signed distance from plane needs to be calculated.
	/// \return The signed distance of the given point from the plane
	T signedDistanceFrom(const Vector3& point)
	{
		return point.dot(m_normal) + m_d;
	}

	/// Normal of the plane.
	Vector3 m_normal;

	/// d from the plane equation (n.x + d = 0).
	T m_d;
};


/// Two ends of triangle edge are given in terms of the following vertex properties.
///		- Signed distance from the colliding triangle.
///		- Projection on the separating axis.
/// Get the intersection of this edge and the plane in terms of the projection of the vertices.
/// \param dStart Signed distance of the start of edge from the plane of the colliding triangle.
/// \param dEnd Signed distance of the end of edge from the plane of the colliding triangle.
/// \param pvStart Projection of the start of edge from the plane of the colliding triangle.
/// \param dStart Signed distance of the first vertex from the plane of the colliding triangle.
template<class T>
void edgeIntersection(T dStart, T dEnd, T pvStart, T pvEnd, T *t, int *tCount)
{
	// Epsilon used in this function.
	static const T EPSILON = T(Geometry::DistanceEpsilon);

	bool startIsUnder = dStart < -EPSILON && dEnd >= -EPSILON;
	bool startIsOver = dStart > EPSILON && dEnd <= EPSILON;

	if (startIsUnder || startIsOver)
	{
		bool isEndOnPlane = startIsUnder ? dEnd < EPSILON : dEnd > -EPSILON;

		if (isEndOnPlane)
		{
			// The intersection of the edge and the plane is the end.
			t[(*tCount)++] = pvEnd;
		}
		else
		{
			// The intersection of the edge and the plane is got by clipping the edge onto the plane.
			t[(*tCount)++] = pvStart + (pvEnd - pvStart) * (dStart / (dStart - dEnd));
		}
	}
}

/// Check if the two triangles intersect using separating axis test.
/// Algorithm is implemented from http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tritri.pdf
///
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t1v0,t1v1,t1v2 Vertices of the first triangle.
/// \param t2v0,t2v1,t2v2 Vertices of the second triangle.
/// \param t1n Normal of the first triangle, should be normalized.
/// \param t2n Normal of the second triangle, should be normalized.
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
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;

	if (t1n.isZero() || t2n.isZero())
	{
		// Degenerate triangle(s) passed to checkTriangleTriangleIntersection.
		return false;
	}

	// Epsilon used in this function.
	static const T EPSILON = T(Geometry::DistanceEpsilon);

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
	Plane<T, MOpt> p1(t1n, t1v0);
	T d2[3] = {p1.signedDistanceFrom(t2v0), p1.signedDistanceFrom(t2v1), p1.signedDistanceFrom(t2v2)};

	if ((d2[0] <= EPSILON && d2[1] <= EPSILON && d2[2] <= EPSILON) ||
		(d2[0] >= -EPSILON && d2[1] >= -EPSILON && d2[2] >= -EPSILON))
	{
		return false;
	}

	// Check if all the vertices of T1 are on one side of p2.
	Plane<T, MOpt> p2(t2n, t2v0);
	T d1[3] = {p2.signedDistanceFrom(t1v0), p2.signedDistanceFrom(t1v1), p2.signedDistanceFrom(t1v2)};

	if ((d1[0] <= EPSILON && d1[1] <= EPSILON && d1[2] <= EPSILON) ||
		(d1[0] >= -EPSILON && d1[1] >= -EPSILON && d1[2] >= -EPSILON))
	{
		return false;
	}

	// The separating axis.
	Eigen::Matrix<T, 3, 1, MOpt> D = t1n.cross(t2n);

	// Projection of the triangle vertices on the separating axis.
	T pv1[3] = {D.dot(t1v0), D.dot(t1v1), D.dot(t1v2)};
	T pv2[3] = {D.dot(t2v0), D.dot(t2v1), D.dot(t2v2)};

	T t[3], s[3];
	int tCount = 0, sCount = 0;

	// Loop through the edges of each triangle and find the intersectio of these edges onto
	// the plane of the other triangle.
	for (int i = 0; i < 3; ++i)
	{
		int j = (i + 1) % 3;

		edgeIntersection(d1[i], d1[j], pv1[i], pv1[j], t, &tCount);
		edgeIntersection(d2[i], d2[j], pv2[i], pv2[j], s, &sCount);
	}

	SURGSIM_ASSERT(tCount == 2 && sCount == 2)
		<< "The intersection between the triangle and the separating axis is not a line segment."
		<< " This scenario cannot happen, at this point in the algorithm.";

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


#endif // SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_INL_H