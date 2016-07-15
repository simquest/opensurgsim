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

#ifndef SURGSIM_MATH_TRIANGLETRIANGLECONTACTCALCULATION_INL_H
#define SURGSIM_MATH_TRIANGLETRIANGLECONTACTCALCULATION_INL_H

namespace SurgSim
{

namespace Math
{

/// A helper class for a triangle, used for the following two purposes:
///		- Clip against a given plane.
///		- Find the deepest point given a plane.
/// Created as a triangle and can become 4 or more sided polygon when clipped.
/// \note The vertices are stored in order, so that the edges of the polygon run between adjacent vertices
///    (and from the last vertex to the first).
/// \tparam T Accuracy of the calculation.
/// \tparam MOpt Eigen Matrix options.
template <class T, int MOpt>
class TriangleHelper
{
	static const size_t CAPACITY = 10;
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	typedef boost::container::static_vector<Vector3, CAPACITY> Vertices;

public:
	/// Constructor using the triangle data to initialize.
	/// \param v0, v1, v2 The vertices of the triangle.
	/// \param n The normal of the triangle.
	TriangleHelper(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& n)
		: m_normal(n), m_receiverBufferIndex(0)
	{
		m_vertices[0] = &v0;
		m_vertices[1] = &v1;
		m_vertices[2] = &v2;
		m_planeD = -m_vertices[0]->dot(m_normal);
	}

	/// Given a triangle, find the deepest vertex in the swept volume of that triangle.
	/// \param triangle The triangle against which the penetration is checked.
	/// \param [out] penetrationDepth The depth of the deepest point in this triangle to the triangle sent in.
	/// \param [out] penetrationPoint0 The penetration point on this triangle.
	/// \param [out] penetrationPoint1 The penetration point on the triangle sent in.
	void findDeepestPenetrationWithTriangle(const TriangleHelper& triangle, T* penetrationDepth,
											Vector3* penetrationPoint0, Vector3* penetrationPoint1)
	{
		m_clippedVerticesBuffer[0].push_back(*m_vertices[0]);
		m_clippedVerticesBuffer[0].push_back(*m_vertices[1]);
		m_clippedVerticesBuffer[0].push_back(*m_vertices[2]);
		m_receiverBufferIndex = 1;

		Vector3 clipPlaneNormal;
		T clipPlaneD;

		for (size_t i = 0; i < 3; ++i)
		{
			triangle.getPrismPlane(i, &clipPlaneNormal, &clipPlaneD);
			clipAgainstPlane(clipPlaneNormal, clipPlaneD);
		}

		findDeepestVertexUnderPlane(triangle.m_normal, triangle.m_planeD, penetrationDepth, penetrationPoint0);

		SURGSIM_ASSERT(*penetrationDepth <= T(0))
				<< "The distance from triangle is calculated as " << *penetrationDepth << ". At this point in the"
				<< " algorithm, the depth is expected to be negative.";

		*penetrationPoint1 = *penetrationPoint0 - (triangle.m_normal * (*penetrationDepth));
		*penetrationDepth = -(*penetrationDepth);
	}

private:
	/// Get the bounding plane of the swept volume of this triangle.
	/// The swept volume of a triangle is an infinitely long prism.
	/// \param index There are three prism sides, the index indicates which one is to be calculated.
	/// \param planeNormal The outward facing normal of the prism plane.
	/// \param planeD d from the plane equation (n.x + d = 0) of the prism plane.
	void getPrismPlane(size_t index, Vector3* planeNormal, T* planeD) const
	{
		*planeNormal = *m_vertices[(index + 1) % 3] - *m_vertices[index];
		*planeNormal = planeNormal->cross(m_normal);
		planeNormal->normalize();
		*planeD = -m_vertices[index]->dot(*planeNormal);
	}

	/// Clip the polygon given a plane. Any part of the polygon above this plane is clipped.
	/// \note This may alter the number of vertices in this polygon.
	/// \param planeN The normal of the clipping plane.
	/// \param planeD The d from plane eqn (nx + d) of the clipping plane.
	void clipAgainstPlane(const Vector3& planeN, T planeD)
	{
		// Loop through the edges starting from (m_vertices[0]->m_vertices[1]) to
		// (m_vertices[m_numVertices - 1]->m_vertices[0]).
		// The start vertex and end vertex can be either under/on/over the clipping plane.
		//		start	|	end		|	action
		// ----------------------------------------------
		//		under	|	under	|	add start to clipped vertices
		//		under	|	on		|	add start to clipped vertices
		//		under	|	over	|	add start to clipped vertices, clip the edge to the plane (creating a vertex)
		//		on		|	under	|	add start to clipped vertices
		//		on		|	on		|	add start to clipped vertices
		//		on		|	over	|	add start to clipped vertices
		//		over	|	under	|	clip the edge to the plane (creating a vertex)
		//		over	|	on		|	none
		//		over	|	over	|	none

		Vertices& clippedVertices = m_clippedVerticesBuffer[m_receiverBufferIndex];
		m_receiverBufferIndex = (m_receiverBufferIndex + 1) % 2;
		Vertices& originalVertices = m_clippedVerticesBuffer[m_receiverBufferIndex];
		clippedVertices.clear();

		static const T EPSILON = T(Geometry::DistanceEpsilon);

		// Calculate the signed distance of the vertices from the clipping plane.
		boost::container::static_vector<T, CAPACITY> signedDistanceFromPlane;
		for (auto it = originalVertices.cbegin(); it != originalVertices.cend(); ++it)
		{
			signedDistanceFromPlane.push_back((*it).dot(planeN) + planeD);
		}

		// Temp variable.
		T ratio;

		// Iterators for the end vertices of an edge.
		typename boost::container::static_vector<Vector3, CAPACITY>::iterator end;

		// Iterators for the signed distance from plane of the start and end vertices of an edge.
		auto startSignedDistance = signedDistanceFromPlane.begin();
		typename boost::container::static_vector<T, CAPACITY>::iterator endSignedDistance;

		// Iterate over the edges of the current polygon.
		for (auto start = originalVertices.begin(); start != originalVertices.end(); ++start, ++startSignedDistance)
		{
			// If the end has reached the end of list, point it back to the front of list.
			end = start + 1;
			endSignedDistance = startSignedDistance + 1;
			if (end == originalVertices.end())
			{
				end = originalVertices.begin();
				endSignedDistance = signedDistanceFromPlane.begin();
			}

			// If the vertex is under or on the plane, add to the clippedVertices.
			if (*startSignedDistance <= EPSILON)
			{
				clippedVertices.push_back(*start);
			}

			// If the edge runs from one side of the plane to another. Clip it.
			if ((*startSignedDistance < -EPSILON && *endSignedDistance > EPSILON) ||
				(*startSignedDistance > EPSILON && *endSignedDistance < -EPSILON))
			{
				ratio = *startSignedDistance / (*startSignedDistance - *endSignedDistance);
				clippedVertices.push_back(*start + (*end - *start) * ratio);
			}
		}
	}

	/// Find the deepest vertex of this polygon under the plane.
	/// \note Asserts if there are no vertices in the polygon.
	/// \param planeN The normal of the plane.
	/// \param planeD The distance from origin of the plane.
	/// \param [out] depth The depth of the deepest point in the polygon from the given plane.
	/// \param [out] point The deepest point in the polgon from the given plane.
	void findDeepestVertexUnderPlane(const Vector3& planeN, T planeD, T* depth, Vector3* point) const
	{
		const Vertices& originalVertices = m_clippedVerticesBuffer[(m_receiverBufferIndex + 1) % 2];

		SURGSIM_ASSERT(originalVertices.size() > 0)
				<< "There are no vertices under the plane. This scenario should not arise according to the"
				<< " Triangle-Triangle Contact Calculation algorithm, because of the circumstances under which"
				<< " this function is set to be called.";

		T signedDistanceFromPlane;
		*depth = T(0);
		for (auto it = originalVertices.cbegin(); it != originalVertices.cend(); ++it)
		{
			signedDistanceFromPlane = (*it).dot(planeN) + planeD;
			if (signedDistanceFromPlane < *depth)
			{
				*depth = signedDistanceFromPlane;
				*point = *it;
			}
		}
	}

	/// Original vertices of the triangle.
	const Vector3* m_vertices[3];

	/// Normal of the triangle.
	const Vector3& m_normal;

	/// d from the plane equation (n.x + d = 0) for the plane of this triangle.
	T m_planeD;

	/// The buffers for the clipped vertices of the triangle.
	Vertices m_clippedVerticesBuffer[2];
	size_t m_receiverBufferIndex;
};


template <class T, int MOpt> inline
bool calculateContactTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0n,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1n,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal)
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;

	// Check if the triangles intersect.
	if (!doesIntersectTriangleTriangle(t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, t0n, t1n))
	{
		return false;
	}

	// When control reaches here, the two triangles are definitely intersecting.
	// Calculate the deepest penetration along each of the triangle normals.

	TriangleHelper<T, MOpt> triangle1(t0v0, t0v1, t0v2, t0n);
	TriangleHelper<T, MOpt> triangle2(t1v0, t1v1, t1v2, t1n);

	// Penetration info to be calculated.
	T penetrationDepths[2] = {T(0), T(0)};
	Vector3 penetrationPoints[2][2];

	// Calculate deepest penetration for each of the triangle.
	triangle1.findDeepestPenetrationWithTriangle(
		triangle2, &penetrationDepths[0], &penetrationPoints[0][0], &penetrationPoints[0][1]);

	triangle2.findDeepestPenetrationWithTriangle(
		triangle1, &penetrationDepths[1], &penetrationPoints[1][1], &penetrationPoints[1][0]);

	// Choose the lower penetration of the two as the contact.
	if (penetrationDepths[0] < penetrationDepths[1])
	{
		*penetrationDepth = penetrationDepths[0];
		*contactNormal = t1n;
		*penetrationPoint0 = penetrationPoints[0][0];
		*penetrationPoint1 = penetrationPoints[0][1];
	}
	else
	{
		*penetrationDepth = penetrationDepths[1];
		*contactNormal = -t0n;
		*penetrationPoint0 = penetrationPoints[1][0];
		*penetrationPoint1 = penetrationPoints[1][1];
	}

	return true;
}


template <class T, int MOpt> inline
bool calculateContactTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal)
{
	Eigen::Matrix<T, 3, 1, MOpt> t0n = (t0v1 - t0v0).cross(t0v2 - t0v0);
	Eigen::Matrix<T, 3, 1, MOpt> t1n = (t1v1 - t1v0).cross(t1v2 - t1v0);
	if (t0n.isZero() || t1n.isZero())
	{
		// Degenerate triangle(s) passed to calculateContactTriangleTriangle
		return false;
	}
	t0n.normalize();
	t1n.normalize();
	return calculateContactTriangleTriangle(t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, t0n, t1n, penetrationDepth,
											penetrationPoint0, penetrationPoint1, contactNormal);
}

template <class T, int MOpt> inline
bool calculateContactTriangleTriangleSeparatingAxis(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0n,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1n,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal)
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	using SurgSim::Math::Geometry::DistanceEpsilon;
	using SurgSim::Math::Geometry::ScalarEpsilon;

	SURGSIM_ASSERT(std::abs(t0n.norm() - 1.0) < ScalarEpsilon && std::abs(t1n.norm() - 1.0) < ScalarEpsilon)
		<< "The normals sent in are not normalized! t0n{" << t0n.transpose() << "}, t1n{" << t1n.transpose() << "}.";

	// Early Rejection test:
	// If all the vertices of one triangle are on one side of the plane of the other triangle,
	// there is no intersection.
	std::array<Vector3, 2> d = {Vector3(t1n.dot(t0v0 - t1v0), t1n.dot(t0v1 - t1v0), t1n.dot(t0v2 - t1v0)),
		Vector3(t0n.dot(t1v0 - t0v0), t0n.dot(t1v1 - t0v0), t0n.dot(t1v2 - t0v0))};

	if ((d[0].array() < DistanceEpsilon).all() || (d[0].array() > -DistanceEpsilon).all() ||
		(d[1].array() < DistanceEpsilon).all() || (d[1].array() > -DistanceEpsilon).all())
	{
		return false;
	}

	const std::array<std::array<Vector3, 3>, 2> tv = {{{t0v0, t0v1, t0v2}, {t1v0, t1v1, t1v2}}};

	// Find the intersection between a triangle and the plane of the other triangle. These intersections would
	// lie on the separating axis (which is the cross product of the triangle normals).
	// The name tsa is to indicate that this is storing the intersection of the Triangle with the Separating Axis.
	std::array<std::array<Vector3, 2>, 2> tsa;
	for (int i = 0; i < 2; ++i)
	{
		int index = 0;

		for (int j = 0; j < 3; ++j)
		{
			int k = (j + 1) % 3;

			// Intersect the edge tivj->tivk with the plane of t{(i+1)/2}
			if ((d[i][j] < 0.0 && d[i][k] >= 0.0) || (d[i][j] > 0.0 && d[i][k] <= 0.0))
			{
				// The edge intersects the plane of t{(i+1)/2}.
				auto ratio = std::abs(d[i][j]) / (std::abs(d[i][j]) + std::abs(d[i][k]));
				tsa[i][index++] = tv[i][j] + ratio * (tv[i][k] - tv[i][j]);
			}
		}

		SURGSIM_ASSERT(index == 2)
			<< "The intersection between the edges of triangle " << i << " and the plane of the other triangle"
			<< " must result in two points exactly.";
	}

	// The separating axis.
	const Vector3 D = t0n.cross(t1n).normalized();
	Vector3 result;

	SURGSIM_ASSERT(distancePointLine(tsa[0][1], tsa[0][0], (tsa[0][0] + D).eval(), &result) < DistanceEpsilon &&
		distancePointLine(tsa[1][0], tsa[0][0], (tsa[0][0] + D).eval(), &result) < DistanceEpsilon &&
		distancePointLine(tsa[1][1], tsa[0][0], (tsa[0][0] + D).eval(), &result) < DistanceEpsilon)
		<< "The intersection points on the triangles do not lie on the separating axis";

	static const int DISTANCE = 0;
	static const int TRIANGLE = 1;
	static const int VERTEX = 2;

	// Find the signed distance of the four points on D (from tsa[0][0])
	// Store it in a tuple containing this signed distance, the corresponding triangle ID and vertex ID.
	std::array<std::tuple<T, int, int>, 4> intervals =
	{std::tuple<T, int, int>(0.0, 0, 0),
	 std::tuple<T, int, int>((tsa[0][1] - tsa[0][0]).dot(D), 0, 1),
	 std::tuple<T, int, int>((tsa[1][0] - tsa[0][0]).dot(D), 1, 0),
	 std::tuple<T, int, int>((tsa[1][1] - tsa[0][0]).dot(D), 1, 1)};

	// Sort the signed distance
	std::sort(intervals.begin(), intervals.end(), [](const std::tuple<T, int, int>& i, std::tuple<T, int, int>& j)
	{
		return (std::get<DISTANCE>(i) < std::get<DISTANCE>(j));
	});

	// The intersections of the triangles on the separating axis (P, Q, R, S) are now sorted according to their distance
	// along the separating axis.
	//
	//   *--------*--------*--------*
	//   P        Q        R        S

	// If P and Q belong to the same triangle, there is no intersection between the triangles.
	enum {P = 0, Q, R, S};
	if (std::get<TRIANGLE>(intervals[P]) == std::get<TRIANGLE>(intervals[Q]))
	{
		return false;
	}

	// At this point, there is some overlap of the triangles along the separating axis.
	size_t indexLeft, indexRight;
	if (std::get<TRIANGLE>(intervals[Q]) == std::get<TRIANGLE>(intervals[R]))
	{
		// Q and R belong to the same triangle: Depending on whether PQ or RS is shorter, either PS is moved to the
		// right of QR or QR is moved to the right of PS.
		//
		//   *--------*--------*--------*
		//   P        Q        R        S
		//            *--------*
		//                t1
		//   *--------------------------*
		//                t2

		if ((std::get<DISTANCE>(intervals[Q]) - std::get<DISTANCE>(intervals[P])) <
			(std::get<DISTANCE>(intervals[S]) - std::get<DISTANCE>(intervals[R])))
		{
			indexLeft = P;
			indexRight = R;
		}
		else
		{
			indexLeft = Q;
			indexRight = S;
		}
	}
	else
	{
		// Q and R belong to different triangle: QS is moved to the right of PR.
		//
		//   *--------*--------*--------*
		//   P        Q        R        S
		//   *-----------------*
		//           t1
		//            *-----------------*
		//                     t2

		indexLeft = Q;
		indexRight = R;
	}

	*penetrationDepth = std::get<DISTANCE>(intervals[indexRight]) - std::get<DISTANCE>(intervals[indexLeft]);
	if (*penetrationDepth < DistanceEpsilon)
	{
		// Not enough overlap to be determined as an intersection.
		return false;
	}

	if (std::get<TRIANGLE>(intervals[indexLeft]) == 0)
	{
		*penetrationPoint0 = tsa[0][std::get<VERTEX>(intervals[indexLeft])];
		*penetrationPoint1 = tsa[1][std::get<VERTEX>(intervals[indexRight])];
	}
	else
	{
		*penetrationPoint0 = tsa[0][std::get<VERTEX>(intervals[indexRight])];
		*penetrationPoint1 = tsa[1][std::get<VERTEX>(intervals[indexLeft])];
	}

	if ((*penetrationPoint0 - *penetrationPoint1).dot(D) > 0.0)
	{
		*contactNormal = -D;
	}
	else
	{
		*contactNormal = D;
	}

	return true;
}


template <class T, int MOpt> inline
bool calculateContactTriangleTriangleSeparatingAxis(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal)
{
	Eigen::Matrix<T, 3, 1, MOpt> t0n = (t0v1 - t0v0).cross(t0v2 - t0v0);
	Eigen::Matrix<T, 3, 1, MOpt> t1n = (t1v1 - t1v0).cross(t1v2 - t1v0);
	if (t0n.isZero() || t1n.isZero())
	{
		// Degenerate triangle(s) passed to calculateContactTriangleTriangle
		return false;
	}
	t0n.normalize();
	t1n.normalize();
	return calculateContactTriangleTriangleSeparatingAxis(t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, t0n, t1n,
		penetrationDepth, penetrationPoint0, penetrationPoint1, contactNormal);
}


} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_TRIANGLETRIANGLECONTACTCALCULATION_INL_H
