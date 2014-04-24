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

#ifndef SURGSIM_MATH_TRIANGLETRIANGLECONTACTCALCULATION_H
#define SURGSIM_MATH_TRIANGLETRIANGLECONTACTCALCULATION_H


#include <boost/container/static_vector.hpp>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/TriangleTriangleIntersection.h"
#include "SurgSim/Math/Vector.h"

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
class TriangleClipper
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	static const size_t CAPACITY = 10;

public:
	/// Constructor using the three vertices to inilialize the polygon.
	/// \param v0, v1, v2 The vertices of the triangle.
	TriangleClipper(const Vector3& v0, const Vector3& v1, const Vector3 &v2)
		: m_numVertices(3)
	{
		m_vertices[0] = v0;
		m_vertices[1] = v1;
		m_vertices[2] = v2;
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
		//		under	|	under	|	none
		//		under	|	on		|	none
		//		under	|	over	|	clip the edge to the plane (creating a vertex)
		//		on		|	under	|	none
		//		on		|	on		|	none
		//		on		|	over	|	none
		//		over	|	under	|	clip the edge to the plane (creating a vertex) and delete start
		//		over	|	on		|	delete start
		//		over	|	over	|	delete start
		//
		// The deletion is not done immediately, as the vertex might still be needed, if it is part of another
		// edge which hasn't been processed yet. So the to-be-deleted vertex indices are kept track of, and
		// deleted at the end.
		// The insertion is done immediately. So the index needs to be incremented to skip over the newly created edge.

		T iSignedDistanceFromPlane = T(0), jSignedDistanceFromPlane = T(0);
		T iDistanceFromPlane = T(0), jDistanceFromPlane = T(0);
		bool iUnderPlane = false, jUnderPlane = false;
		bool iOverPlane = false, jOverPlane = false;
		size_t toBeDeletedVerticesList[CAPACITY];
		size_t toBeDeletedVerticesCount = 0;
		T e = T(1e-10);

		for (size_t i = 0, j = 0; i < m_numVertices; ++i)
		{
			j = (i + 1) % m_numVertices;

			iSignedDistanceFromPlane = m_vertices[i].dot(planeN) + planeD;
			jSignedDistanceFromPlane = m_vertices[j].dot(planeN) + planeD;

			iUnderPlane = iSignedDistanceFromPlane < -e;
			jUnderPlane = jSignedDistanceFromPlane < -e;
			iOverPlane = iSignedDistanceFromPlane > e;
			jOverPlane = jSignedDistanceFromPlane > e;

			if (iUnderPlane && jOverPlane)
			{
				iDistanceFromPlane = std::abs(iSignedDistanceFromPlane);
				jDistanceFromPlane = std::abs(jSignedDistanceFromPlane);
				insertVertex(i + 1, m_vertices[i] + (m_vertices[j] - m_vertices[i]) *
													(iDistanceFromPlane / (iDistanceFromPlane + jDistanceFromPlane)));
				++i;
			}
			else if (iOverPlane && jUnderPlane)
			{
				iDistanceFromPlane = std::abs(iSignedDistanceFromPlane);
				jDistanceFromPlane = std::abs(jSignedDistanceFromPlane);
				insertVertex(i + 1, m_vertices[i] + (m_vertices[j] - m_vertices[i]) *
													(iDistanceFromPlane / (iDistanceFromPlane + jDistanceFromPlane)));
				toBeDeletedVerticesList[toBeDeletedVerticesCount++] = i;
				++i;
			}
			else if (iOverPlane)
			{
				toBeDeletedVerticesList[toBeDeletedVerticesCount++] = i;
			}
		}

		for (size_t i = 0; i < toBeDeletedVerticesCount; ++i)
		{
			deleteVertex(toBeDeletedVerticesList[i] - i);
		}
	}

	/// Find the deepest vertex of this polygon under the plane.
	/// \note Asserts if there are no vertices in the polygon.
	/// \param planeN The normal of the plane.
	/// \param planeD The distance from origin of the plane.
	void findDeepestVertexUnderPlane(const Vector3& planeN, T planeD, T* depth, Vector3* point) const
	{
		SURGSIM_ASSERT(m_numVertices > 0)
			<< "There are no vertices under the plane. This scenario should not arise according to the"
			<< " Triangle-Triangle Contact Calculation algorithm, because of the circumstances under which"
			<< " this function is set to be called.";

		T signedDistanceFromPlane = T(0);
		*depth = T(0);
		for (size_t i = 0; i < m_numVertices; ++i)
		{
			signedDistanceFromPlane = m_vertices[i].dot(planeN) + planeD;
			if (signedDistanceFromPlane < *depth)
			{
				*depth = signedDistanceFromPlane;
				*point = m_vertices[i];
			}
		}
	}

private:
	/// Insert a vertex at the given vertex position (including the end of the array).
	/// Since the vertices are ordered, the insertion of a vertex moves all the vertices from that position to the end
	/// of the list to be moved one slot down.
	/// \note Asserts if polygon capacity is full.
	/// \note Asserts if insert position is invalid.
	/// \param position The position at which the new vertex is inserted.
	/// \param vertex The new vertex to be inserted to the polygon.
	void insertVertex(size_t position, const Vector3& vertex)
	{
		SURGSIM_ASSERT(m_numVertices < CAPACITY)
			<< "Polygon (of capacity " << CAPACITY << ") cannot hold any more vertices.";
		SURGSIM_ASSERT(position >= 0 && position <= m_numVertices)
			<< "Invalid insert position (" << position << ") for polygon of size " << m_numVertices << ".";

		int size = m_numVertices - position;
		if (size > 0)
		{
			std::memmove(&m_vertices[position + 1], &m_vertices[position], sizeof(Vector3) * size);
		}

		m_vertices[position] = vertex;
		m_numVertices++;
	}

	/// Delete the vertex at the given vertex position.
	/// \note Asserts if deletion position is invalid.
	/// \param position The position where the vertex is deleted.
	void deleteVertex(size_t position)
	{
		SURGSIM_ASSERT(position >= 0 && position < m_numVertices)
			<< "Invalid delete position (" << position << ") for polygon of size " << m_numVertices << ".";

		int size = m_numVertices - position - 1;
		if (size > 0)
		{
			std::memmove(&m_vertices[position], &m_vertices[position + 1], sizeof(Vector3) * size);
		}

		m_numVertices--;
	}

	/// The vertices of this polygon.
	Vector3 m_vertices[CAPACITY];

	/// The current number of vertices in this polygon.
	size_t m_numVertices;
};


/// Calculate the contact between two triangles.
/// Algorithm presented in https://docs.google.com/a/simquest.com/document/d/11ajMD7QoTVelT2_szGPpeUEY0wHKKxW1TOgMe8k5Fsc.
/// If the triangle are known to intersect, the deepest penetration of the triangle into each other is calculated.
/// The triangle which penetrates less into the other triangle is chosen as contact.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Vertices of the first triangle.
/// \param t1v0,t1v1,t1v2 Vertices of the second triangle.
/// \param t0n Unit length normal of the first triangle.
/// \param t1n Unit length normal of the second triangle.
/// \param [out] depth The depth of penetration.
/// \param [out] penetrationPoint0 The contact point on triangle0 (t0v0,t0v1,t0v2).
/// \param [out] penetrationPoint1 The contact point on triangle1 (t1v0,t1v1,t1v2).
/// \param [out] normal The contact normal that points from triangle1 to triangle0.
/// \return True, if intersection is detected.
/// \note The [out] params are not modified if there is no intersection.
/// \note If penetrationPoint0 is moved by -(nomal*depth*0.5) and penetrationPoint1 is moved by (nomal*depth*0.5), the
/// triangles will no longer be intersecting.
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
	T *depth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* normal)
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;

	// Check for degenerate triangle.
	if (t0n.isZero() || t1n.isZero())
	{
		return false;
	}

	// Check if the triangles intersect.
	if (!checkTriangleTriangleIntersection(t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, t0n, t1n))
	{
		return false;
	}

	// Epsilon used in this function.
	T e = T(1e-10);

	// When control reaches here, the two triangles are definitely intersecting.
	// Calculate the deepest penetration along each of the triangle normals.

	// Triangle info in arrays.
	const Vector3 v[2][3] = {{t0v0, t0v1, t0v2}, {t1v0, t1v1, t1v2}};
	const Vector3 n[2] = {t0n, t1n};

	// d from the plane eqn. of the triangles (y = nx + d).
	const T d[2] = {-t0v0.dot(t0n), -t1v0.dot(t1n)};

	// Penetration info to be calculated.
	T penetrationDepth[2] = {T(0), T(0)};
	Vector3 penetrationPoint[2][2];

	// Calculate deepest penetration for each of the triangle.
	for (int i = 0, j = 1; i < 2; ++i, --j)
	{
		// Index 'i' is the triangle which is checked against the plane of triangle
		// with index 'j' for deepest penetration.

		TriangleClipper<T, MOpt> clipper(v[i][0], v[i][1], v[i][2]);

		// The clipping planes, are:
		// 1 - Plane perpendicular to the plane of triangle 'j' and containing v[j][0] and v[j][1].
		// 2 - Plane perpendicular to the plane of triangle 'j' and containing v[j][1] and v[j][2].
		// 3 - Plane perpendicular to the plane of triangle 'j' and containing v[j][2] and v[j][0].

		// For all of these planes, we need to calculate the normal and signed distance from origin.
		Vector3 clipPlaneNormal;

		for (int k = 0; k < 3; k++)
		{
			clipPlaneNormal = (v[j][(k + 1) % 3] - v[j][k]).cross(n[j]);
			clipPlaneNormal.normalize();
			clipper.clipAgainstPlane(clipPlaneNormal, -v[j][k].dot(clipPlaneNormal));
		}

		clipper.findDeepestVertexUnderPlane(n[j], d[j], &penetrationDepth[i], &penetrationPoint[i][i]);
		SURGSIM_ASSERT(penetrationDepth[i] < T(0))
			<< "Deepest penetration not calculated for a confirmed triangle-triangle intersection";

		penetrationPoint[i][j] = penetrationPoint[i][i] - (n[j] * penetrationDepth[i]);
		penetrationDepth[i] = std::abs(penetrationDepth[i]);
	}

	// Choose the lower penetration of the two as the contact.
	if (penetrationDepth[0] < penetrationDepth[1])
	{
		*depth = penetrationDepth[0];
		*normal = t1n;
		*penetrationPoint0 = penetrationPoint[0][0];
		*penetrationPoint1 = penetrationPoint[0][1];
	}
	else
	{
		*depth = penetrationDepth[1];
		*normal = -t0n;
		*penetrationPoint0 = penetrationPoint[1][0];
		*penetrationPoint1 = penetrationPoint[1][1];
	}

	return true;
}


/// Calculate the contact between two triangles.
/// Algorithm presented in https://docs.google.com/a/simquest.com/document/d/11ajMD7QoTVelT2_szGPpeUEY0wHKKxW1TOgMe8k5Fsc.
/// If the triangle are known to intersect, the deepest penetration of the triangle into each other is calculated.
/// The triangle which penetrates less into the other triangle is chosen as contact.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param t0v0,t0v1,t0v2 Vertices of the first triangle.
/// \param t1v0,t1v1,t1v2 Vertices of the second triangle.
/// \param t0n Unit length normal of the first triangle.
/// \param t1n Unit length normal of the second triangle.
/// \param [out] depth The depth of penetration.
/// \param [out] penetrationPoint0 The contact point on triangle0 (t0v0,t0v1,t0v2).
/// \param [out] penetrationPoint1 The contact point on triangle1 (t1v0,t1v1,t1v2).
/// \param [out] normal The contact normal that points from triangle1 to triangle0.
/// \return True, if intersection is detected.
/// \note The [out] params are not modified if there is no intersection.
/// \note If penetrationPoint0 is moved by -(nomal*depth*0.5) and penetrationPoint1 is moved by (nomal*depth*0.5), the
/// triangles will no longer be intersecting.
template <class T, int MOpt> inline
bool calculateContactTriangleTriangle(
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t0v2,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v0,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v1,
	const Eigen::Matrix<T, 3, 1, MOpt>& t1v2,
	T *depth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* normal)
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
	return calculateContactTriangleTriangle(t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, t0n, t1n, depth, penetrationPoint0,
											penetrationPoint1, normal);
}


} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_TRIANGLETRIANGLECONTACTCALCULATION_H