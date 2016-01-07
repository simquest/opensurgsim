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

namespace
{
static const double EPSILOND = 1e-12;
}

namespace SurgSim
{

namespace Math
{


/// Two ends of the triangle edge are given in terms of the following vertex properties.
///		- Signed distance from the colliding triangle.
///		- Projection on the separating axis.
/// Get the intersection of this edge and the plane in terms of the projection on the separating axis.
/// \tparam T Accuracy of the calculation, can usually be inferred.
/// \param dStart Signed distance of the start of edge from the plane of the colliding triangle.
/// \param dEnd Signed distance of the end of edge from the plane of the colliding triangle.
/// \param pvStart Projection of the start of edge from the plane of the colliding triangle.
/// \param pvEnd Projection of the end of edge from the plane of the colliding triangle.
/// \param parametricIntersection Parametric representation of the intersection between the triangle edge
///		   and the plane in terms of the projection on the separating axis.
/// \param parametricIntersectionIndex The array index of parametricIntersection.
template<class T>
void edgeIntersection(T dStart, T dEnd, T pvStart, T pvEnd, T* parametricIntersection,
					  size_t* parametricIntersectionIndex)
{
	// Epsilon used in this function.
	static const T EPSILON = static_cast<T>(EPSILOND);

	bool edgeFromUnderToAbove = dStart < 0.0 && dEnd >= 0.0;
	bool edgeFromAboveToUnder = dStart > 0.0 && dEnd <= 0.0;

	if (edgeFromUnderToAbove || edgeFromAboveToUnder)
	{
		if (std::abs(dStart - dEnd) < EPSILON)
		{
			// Start and End are really close. Pick start.
			parametricIntersection[(*parametricIntersectionIndex)++] = pvStart;
		}
		else
		{
			// Clip to the point in the intersection of Start->End and plane of the colliding triangle.
			parametricIntersection[(*parametricIntersectionIndex)++] =
				pvStart + (pvEnd - pvStart) * (dStart / (dStart - dEnd));
		}
	}
}

template <class T, int MOpt> inline
bool doesIntersectTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0n,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1n)
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	using SurgSim::Math::Geometry::DistanceEpsilon;

	if (t0n.isZero() || t1n.isZero())
	{
		// Degenerate triangle(s) passed to checkTriangleTriangleIntersection.
		return false;
	}

	// Variable names mentioned here are the notations used in the paper:
	// T1		- Triangle with vertices (t0v0, t0v1, t0v2).
	// T2		- Triangle with vertices (t1v0, t1v1, t1v2).
	// d1[3]	- Signed distances of the vertices of T1 from the plane of T2.
	// d2[3]	- Signed distances of the vertices of T2 from the plane of T1.
	// D		- Separating axis used for the test. This is calculated as the cross products of the triangle normals.
	// pv1[3]	- Projection of the vertices of T1 onto the separating axis (D).
	// pv2[3]	- Projection of the vertices of T2 onto the separating axis (D).
	// s1[2]	- The intersection between T1 and D is a line segment.
	//			  s1[0] and s1[1] are the parametric representation of the ends of this line segment.
	// s2[2]	- The intersection between T2 and D is a line segment.
	//			  s2[0] and s2[1] are the parametric representation of the ends of this line segment.

	// Early Rejection test:
	// If all the vertices of one triangle are on one side of the plane of the other triangle,
	// there is no intersection.

	// Check if all the vertices of T2 are on one side of p1.
	// Plane eqn of T1: DotProduct(t0n, X) + distanceFromOrigin = 0
	// where distanceFromOrigin = -DotProduct(t0n, t0v0)
	// So, plane eqn of T1: DotProduct(t0n, X - t0v0) = 0
	// Distance of first vertex of T2 from the plane of T1 is: DotProduct(t0n, t1v0 - t0v0)
	Vector3 d2(t0n.dot(t1v0 - t0v0), t0n.dot(t1v1 - t0v0), t0n.dot(t1v2 - t0v0));

	if ((d2.array() < DistanceEpsilon).all() || (d2.array() > -DistanceEpsilon).all())
	{
		return false;
	}

	// Check if all the vertices of T1 are on one side of p2.
	Vector3 d1(t1n.dot(t0v0 - t1v0), t1n.dot(t0v1 - t1v0), t1n.dot(t0v2 - t1v0));

	if ((d1.array() < DistanceEpsilon).all() || (d1.array() > -DistanceEpsilon).all())
	{
		return false;
	}

	// The separating axis.
	Vector3 D = t0n.cross(t1n);

	// Projection of the triangle vertices on the separating axis.
	Vector3 pv1(D.dot(t0v0), D.dot(t0v1), D.dot(t0v2));
	Vector3 pv2(D.dot(t1v0), D.dot(t1v1), D.dot(t1v2));

	// The intersection of the triangles with the separating axis (D).
	T s1[3];
	T s2[3];
	size_t s1Index = 0;
	size_t s2Index = 0;

	// Loop through the edges of each triangle and find the intersection of these edges onto
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

	// s1[0], s1[1] are the (unordered) extents of the projection of T1 on D.
	// s2[0], s2[1] are the (unordered) extents of the projection of T2 on D.
	// If both these are line segments (i.e. the distance between them is > epsilon),
	// and if they overlap, then the two triangles intersect.

	return !(std::abs(s1[0] - s1[1]) <= DistanceEpsilon || std::abs(s2[0] - s2[1]) <= DistanceEpsilon) &&
		   !(s1[0] <= (s2[0] + DistanceEpsilon) && s1[0] <= (s2[1] + DistanceEpsilon) &&
			 s1[1] <= (s2[0] + DistanceEpsilon) && s1[1] <= (s2[1] + DistanceEpsilon)) &&
		   !(s1[0] >= (s2[0] - DistanceEpsilon) && s1[0] >= (s2[1] - DistanceEpsilon) &&
			 s1[1] >= (s2[0] - DistanceEpsilon) && s1[1] >= (s2[1] - DistanceEpsilon));
}


template <class T, int MOpt> inline
bool doesIntersectTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2)
{
	Eigen::Matrix<T, 3, 1, MOpt> t0n = (t0v1 - t0v0).cross(t0v2 - t0v0);
	Eigen::Matrix<T, 3, 1, MOpt> t1n = (t1v1 - t1v0).cross(t1v2 - t1v0);
	if (t0n.isZero() || t1n.isZero())
	{
		// Degenerate triangle(s) passed to checkTriangleTriangleIntersection.
		return false;
	}
	t0n.normalize();
	t1n.normalize();
	return doesIntersectTriangleTriangle(t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, t0n, t1n);
}


} // namespace Math

} // namespace SurgSim


#endif // SURGSIM_MATH_TRIANGLETRIANGLEINTERSECTION_INL_H
