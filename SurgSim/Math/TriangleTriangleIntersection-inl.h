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
	/// Contructor takes in the plane normal and a point on the plane.
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


/// Two ends of the triangle edge are given in terms of the following vertex properties.
///		- Signed distance from the colliding triangle.
///		- Projection on the separating axis.
/// Get the intersection of this edge and the plane in terms of the projection on the separating axis.
/// \tparam T Accuracy of the calculation, can usually be inferred.
/// \param dStart Signed distance of the start of edge from the plane of the colliding triangle.
/// \param dEnd Signed distance of the end of edge from the plane of the colliding triangle.
/// \param pvStart Projection of the start of edge from the plane of the colliding triangle.
/// \param dStart Signed distance of the first vertex from the plane of the colliding triangle.
/// \param parametricIntersection Parametric representation of the intersection between the triangle edge
///		   and the plane in terms of the projection on the separating axis.
/// \param parametricIntersectionIndex The array index of parametricIntersection.
template<class T>
void edgeIntersection(T dStart, T dEnd, T pvStart, T pvEnd, T* parametricIntersection,
					  size_t* parametricIntersectionIndex)
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
			parametricIntersection[(*parametricIntersectionIndex)++] = pvEnd;
		}
		else
		{
			// The intersection of the edge and the plane is got by clipping the edge onto the plane.
			parametricIntersection[(*parametricIntersectionIndex)++] =
				pvStart + (pvEnd - pvStart) * (dStart / (dStart - dEnd));
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
	// s1[2]	- The intersection between T1 and D is a line segment.
	//			  s1[0] and s1[1] are the parametric representation of the ends of this line segment.
	// s2[2]	- The intersection between T2 and D is a line segment.
	//			  s2[0] and s2[1] are the parametric representation of the ends of this line segment.

	// Some extra variables:
	// p1		- The plane of the triangle T1.
	// p2		- The plane of the triangle T2.

	// Early Rejection test:
	// If all the vertices of one triangle are on one side of the plane of the other triangle,
	// there is no intersection.

	// Check if all the vertices of T2 are on one side of p1.
	Plane<T, MOpt> p1(t1n, t1v0);
	Vector3 d2(p1.signedDistanceFrom(t2v0), p1.signedDistanceFrom(t2v1), p1.signedDistanceFrom(t2v2));

	if ((d2.array() <= EPSILON).all() || (d2.array() >= -EPSILON).all())
	{
		return false;
	}

	// Check if all the vertices of T1 are on one side of p2.
	Plane<T, MOpt> p2(t2n, t2v0);
	Vector3 d1(p2.signedDistanceFrom(t1v0), p2.signedDistanceFrom(t1v1), p2.signedDistanceFrom(t1v2));

	if ((d1.array() <= EPSILON).all() || (d1.array() >= -EPSILON).all())
	{
		return false;
	}

	// The separating axis.
	Vector3 D = t1n.cross(t2n);

	// Projection of the triangle vertices on the separating axis.
	Vector3 pv1(D.dot(t1v0), D.dot(t1v1), D.dot(t1v2));
	Vector3 pv2(D.dot(t2v0), D.dot(t2v1), D.dot(t2v2));

	// The intersection of the triangles with the separating axis (D).
	T s1[3];
	T s2[3];
	size_t s1Index = 0;
	size_t s2Index = 0;

	// Loop through the edges of each triangle and find the intersectio of these edges onto
	// the plane of the other triangle.
	for (int i = 0; i < 3; ++i)
	{
		int j = (i + 1) % 3;

		edgeIntersection(d1[i], d1[j], pv1[i], pv1[j], s1, &s1Index);
		edgeIntersection(d2[i], d2[j], pv2[i], pv2[j], s2, &s2Index);
	}

	SURGSIM_ASSERT(s1Index == 2 && s2Index == 2)
			<< "The intersection between the triangle and the separating axis is not a line segment."
			<< " This scenario cannot happen, at this point in the algorithm.";

	return !(s1[0] <= s2[0] && s1[0] <= s2[1] && s1[1] <= s2[0] && s1[1] <= s2[1]) &&
		   !(s1[0] >= s2[0] && s1[0] >= s2[1] && s1[1] >= s2[0] && s1[1] >= s2[1]);
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