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

#ifndef SURGSIM_MATH_TRIANGLECAPSULECONTACTCALCULATION_INL_H
#define SURGSIM_MATH_TRIANGLECAPSULECONTACTCALCULATION_INL_H

#include "SurgSim/Math/Valid.h"
#include "SurgSim/DataStructures/OptionalValue.h"

namespace SurgSim
{

namespace Math
{

namespace TriangleCapsuleContactCalculation
{

/// Class used to find the point on (positive X side of) ellipse with the given slope.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
template <class T, int MOpt>
class EllipseHelper
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> RigidTransform3;

public:
	/// \param center Center of the ellipse.
	/// \param majorAxis, minorAxis The major/minor axes of the ellipse, both of unit length
	/// \param majorRadius, minorRadius Major/minor radii of the ellipse
	/// \note majorAxis and minorAxis are assumed to be orthogonal to each other.
	EllipseHelper(const Vector3& center, const Vector3& majorAxis, const Vector3& minorAxis, const double majorRadius,
		const double minorRadius) : a(majorRadius), b(minorRadius)
	{
		m_transform.translation() = center;
		m_transform.linear().col(0) = majorAxis;
		m_transform.linear().col(1) = minorAxis;
		m_transform.linear().col(2) = majorAxis.cross(minorAxis);
		m_inverseTransform = m_transform.inverse();
	}

	/// \param tangent The given tangent to this ellipse, whose corresponding point is to be found
	/// \return The point on the ellipse (in positive x direction) which has the given tangent.
	Vector3 pointWithTangent(const Vector3& tangent)
	{
		// tangent in local coordinates.
		Vector localTangent = m_inverseTransform.linear() * tangent;

		// Slope of this tangent
		T m = localTangent[1] / localTangent[0];

		// Ellipse equation: x*x/a*a + y*y/b*b = 1
		// Rewriting ellipse equation in the form, y = f(x): y = sqrt(a*a - x*x) * b / a
		// Slope of ellipse: y' = -x*b*b/a*a*y
		// This slope is equal to the slope of the localTangent. So, we can solve for x and y.
		T x = std::sqrt((m * m * a * a * a * a) / (b * b + m * m * a * a));
		T y = (b / a) * std::sqrt(a * a - x * x) * ((m > 0.0) ? -1.0 : 1.0);

		// Transforming this point into world coordinates.
		return m_transform * Vector3(x, y, static_cast<T>(0));
	}

private:
	/// Major radius of the ellipse
	double a;
	/// Minor radius of the ellipse
	double b;
	/// Transform local ellipse coordinates to world coordinates
	RigidTransform3 m_transform;
	/// Transform world coordinates to local coordinates
	RigidTransform3 m_inverseTransform;
};

/// Class used to find the intersection between a triangle and a capsule.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
template <class T, int MOpt>
class TriangleCapsuleContactCalculation
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	typedef Eigen::Matrix<T, 2, 1, MOpt> Vector2;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> RigidTransform3;

public:
	/// Constructor.
	/// \param tv0,tv1,tv2 Vertices of the triangle.
	/// \param tn Normal of the triangle, should be normalized.
	/// \param cv0,cv1 Ends of the capsule axis.
	/// \param cr Capsule radius.
	/// \param [out] penetrationDepth The depth of penetration.
	/// \param [out] penetrationPointTriangle The contact point on triangle.
	/// \param [out] penetrationPointCapsule The contact point on capsule.
	/// \param [out] contactNormal The contact normal that points from capsule to triangle.
	/// \param [out] penetrationPointCapsuleAxis The point on the capsule axis closest to the triangle.
	TriangleCapsuleContactCalculation(
		const Vector3& tv0, const Vector3& tv1, const Vector3& tv2,
		const Vector3& tn,
		const Vector3& cv0, const Vector3& cv1,
		double cr,
		T* penetrationDepth,
		Vector3* penetrationPointTriangle,
		Vector3* penetrationPointCapsule,
		Vector3* contactNormal,
		Vector3* penetrationPointCapsuleAxis)
		: m_tv0(tv0), m_tv1(tv1), m_tv2(tv2), m_tn(tn),
		  m_cvTop(cv0), m_cvBottom(cv1), m_cr(cr),
		  m_penetrationDepth(penetrationDepth),
		  m_penetrationPointTriangle(penetrationPointTriangle),
		  m_penetrationPointCapsule(penetrationPointCapsule),
		  m_contactNormal(contactNormal),
		  m_penetrationPointCapsuleAxis(penetrationPointCapsuleAxis)
	{
		m_epsilon = static_cast<T>(Geometry::DistanceEpsilon);
		m_distance = distanceSegmentTriangle(cv0, cv1, m_tv0, m_tv1, m_tv2, m_tn,
			m_penetrationPointCapsuleAxis, m_penetrationPointTriangle);
		m_cAxis = m_cvBottom - m_cvTop;
		m_cLength = m_cAxis.norm();
		m_cAxis = m_cAxis / m_cLength;
		if (m_cAxis.dot(tn) > 0.0)
		{
			m_cvTop = cv1;
			m_cvBottom = cv0;
			m_cAxis = -m_cAxis;
		}
	}

	/// \return Whether there is an intersection.
	bool isIntersecting()
	{
		return m_distance < (m_cr - m_epsilon);
	}

	/// Calculate the contact info.
	void calculateContact()
	{
		if (isIntersecting())
		{
			bool result =
				axisAwayFromTriangle() ||
				axisPerpendicularToTriangle() ||
				axisTouchingTriangle() ||
				axisThroughTriangle();

			SURGSIM_ASSERT(result) << "At this point, there has to be an intersection.";
		}
	}

private:
	/// This function handles the contact data calculation for the case where there is an intersection between the
	/// capsule and the triangle, but the capsule axis does not intersect the triangle.
	/// \return True, if the axis of the capsule is away from the triangle.
	/// \note This function presupposes that isIntersecting() returned true.
	bool axisAwayFromTriangle()
	{
		if (m_distance > m_epsilon)
		{
			*m_contactNormal = *m_penetrationPointTriangle - *m_penetrationPointCapsuleAxis;
			m_contactNormal->normalize();
			*m_penetrationPointCapsule = *m_penetrationPointCapsuleAxis + (*m_contactNormal * m_cr);
			*m_penetrationDepth = m_cr - m_distance;
			return true;
		}

		return false;
	}

	/// This function handles the contact data calculation for the case where there is an intersection between the
	/// capsule and the triangle, and the capsule axis is perpendicular to the triangle.
	/// \return True, if the axis of the capsule is perpendicular to the triangle.
	/// \note This function presupposes that isIntersecting() returned true and axisAwayFromTriangle() returned false.
	bool axisPerpendicularToTriangle()
	{
		if (std::abs(m_cAxis.dot(m_tn) + 1.0) < m_epsilon)
		{
			*m_penetrationPointCapsule = m_cvBottom - m_tn * m_cr;
			*m_penetrationPointCapsuleAxis = m_cvBottom;
			*m_contactNormal = -m_tn;
			*m_penetrationDepth = (m_tv0 - *m_penetrationPointCapsule).dot(m_tn);
			return true;
		}

		return false;
	}

	/// This function handles the contact data calculation for the case where there is an intersection between the
	/// capsule and the triangle, and the capsule axis just touches the triangle.
	/// \return True, if the axis of the capsule is just touching triangle.
	/// \note This function presupposes that isIntersecting() returned true, axisAwayFromTriangle() returned false,
	///		  and axisPerpendicularToTriangle() returned false.
	bool axisTouchingTriangle()
	{
		if (m_penetrationPointCapsuleAxis->isApprox(m_cvTop, m_epsilon) ||
			m_penetrationPointCapsuleAxis->isApprox(m_cvBottom, m_epsilon) ||
			isPointOnTriangleEdge(*m_penetrationPointTriangle, m_tv0, m_tv1, m_tv2, m_tn))
		{
			*m_contactNormal = -m_tn;

			auto projectionCvBottom = (m_cvBottom + m_tn * (m_tv0 - m_cvBottom).dot(m_tn)).eval();
			if (SurgSim::Math::isPointInsideTriangle(projectionCvBottom, m_tv0, m_tv1, m_tv2, m_tn))
			{
				*m_contactNormal = -m_tn;
				*m_penetrationPointCapsule = m_cvBottom - m_tn * m_cr;
				*m_penetrationPointCapsuleAxis = m_cvBottom;
			}
			else
			{
				farthestIntersectionLineCapsule(*m_penetrationPointTriangle, -m_tn,
					m_penetrationPointCapsule, m_penetrationPointCapsuleAxis);
			}
			*m_penetrationDepth = (m_tv0 - *m_penetrationPointCapsule).dot(m_tn);
			return true;
		}

		return false;
	}

	/// This function handles the contact data calculation for the case where there is an intersection between the
	/// capsule and the triangle, and the capsule axis goes through the triangle.
	/// \return True, if the axis of the capsule goes through the triangle. Also calculates the contact info.
	/// \note This function presupposes that isIntersecting() returned true, axisAwayFromTriangle() returned false,
	///		  axisPerpendicularToTriangle() returned false, and axisTouchingTriangle() returned false.
	bool axisThroughTriangle()
	{
		if (m_distance != 0.0)
		{
			return false;
		}

		// Extruding the triangle along the direction of -tn creates a volume. Clip off the capsule axis that is
		// outside of this volume.
		Vector3 v[3] = {m_tv0, m_tv1, m_tv2};
		Vector3 planeN[4] = {m_tn, (-m_tn.cross(m_tv1 - m_tv0)).normalized(), (-m_tn.cross(m_tv2 - m_tv1)).normalized(),
			(-m_tn.cross(m_tv0 - m_tv2)).normalized()};
		double planeD[4] = {-m_tv0.dot(planeN[0]), -m_tv0.dot(planeN[1]), -m_tv1.dot(planeN[2]), -m_tv2.dot(planeN[3])};
		auto segmentStart = m_cvTop;
		auto segmentEnd = m_cvBottom;

		size_t j = clipSegmentAgainstTriangle(&segmentStart, &segmentEnd, v, planeN, planeD);

		if (j == 0)
		{
			*m_contactNormal = -m_tn;
			*m_penetrationPointCapsule = m_cvBottom - m_tn * m_cr;
			*m_penetrationDepth = (m_tv0 - *m_penetrationPointCapsule).dot(m_tn);
			*m_penetrationPointCapsuleAxis = m_cvBottom;
			*m_penetrationPointTriangle = *m_penetrationPointCapsule + m_tn * *m_penetrationDepth;
			return true;
		}

		Vector3 deepestPoint = ((segmentStart - segmentEnd).dot(planeN[0]) < 0.0) ? segmentStart : segmentEnd;
		Vector3 edgeVertices[2] = {v[(j - 1) % 3], v[j % 3]};
		Vector3 triangleEdge = (edgeVertices[1] - edgeVertices[0]).normalized();

		// The capsule axis intersects the plane (planeN[j], planeD[j]). First, the capsule is treated as a cylinder.
		// Its intersection with this plane is an ellipse. The deepest point on this ellipse along -tn and bounded
		// by vectors (edgeVertices[0] -> edgeVertices[0] - tn) and (edgeVertices[1] -> edgeVertices[1] - tn) is found.
		Vector3 center = deepestPoint;
		Vector3 majorAxis = (triangleEdge * (triangleEdge.dot(m_cAxis)) + m_tn * (m_tn.dot(m_cAxis))).normalized();
		double majorRadius = farthestIntersectionLineCylinder(center, majorAxis, &deepestPoint);
		SURGSIM_ASSERT(isValid(majorRadius)) << "The major radius of the ellipse should be a valid number.";

		if (std::abs(majorAxis.dot(triangleEdge)) > m_epsilon)
		{
			// majorApex is not the deepest point because the ellipse is angled. The deepest point is between majorApex
			// and minorApex on the circumference of the ellipse, and the tangent at that point is parallel to the
			// triangleEdge.
			auto minorAxis = planeN[j].cross(majorAxis);
			double minorRadius = farthestIntersectionLineCylinder(center, minorAxis);
			SURGSIM_ASSERT(isValid(minorRadius)) << "The minor radius of the ellipse should be a valid number.";

			EllipseHelper<T, MOpt> ellipseHelper(center, majorAxis, minorAxis, majorRadius, minorRadius);
			deepestPoint = ellipseHelper.pointWithTangent(triangleEdge);
			Vector3 result;
			if (std::abs(distancePointSegment(deepestPoint, m_cvTop, m_cvBottom, &result) - m_cr) > m_epsilon)
			{
				// The deepest point in not on the capsule, which means that the capsule end (the sphere) is
				// intersecting the triangle edge plane (planeN[j], planeD[j]). The intersection between them is a
				// circle. Define a 2D co-ordinate system with the origin at edgeVertices[0], the x-axis as
				// triangleEdge, and the y-axis as tn. Transforming the circle to this 2D co-ordinate system, creates a
				// circle of radius, r, with its center at x, y. Now the deepest point on this circle is (x, y - r).
				Vector3 origin = edgeVertices[0], xAxis = triangleEdge, yAxis = m_tn, zAxis = planeN[j];
				
				double sphereCenterToXYPlane = (m_cvBottom - origin).dot(zAxis);
				double circleRadius = std::sqrt(m_cr * m_cr - sphereCenterToXYPlane * sphereCenterToXYPlane);
				SURGSIM_ASSERT(isValid(circleRadius))
					<< "The radius of the circle of intersection between the sphere and the plane is invalid.";
				Vector3 circleCenter = m_cvBottom - zAxis * sphereCenterToXYPlane;
				double x = (circleCenter - origin).dot(xAxis);
				double y = (circleCenter - origin).dot(yAxis) - circleRadius;
				deepestPoint = xAxis * x + yAxis * y + origin;
			}
		}

		// Project deepestPoint on the triangle edge to make sure it is within the edge.
		auto edgeLength = (edgeVertices[1] - edgeVertices[0]).norm();
		double deepestPointDotEdge = triangleEdge.dot(deepestPoint - edgeVertices[0]);
		if (deepestPointDotEdge <= -m_epsilon || deepestPointDotEdge >= edgeLength + m_epsilon)
		{
			// In this case, the intersection of the cylinder with the triangle edge plane gives an ellipse
			// that is close to a triangle corner and the deepest penetration point on the ellipse is
			// actually outside the triangle.
			// Solution: find the deepest point on the ellipse which projection is still on the triangle.
			// For that: Find the corner edgeVertex of the triangle closest to deepestPointDotEdge
			// Find the deepest penetration point verifying P - tn*t and the cylinder equation.

			// The triangle point to consider is edgeVertices[0] or edgeVertices[1].
			Vector3 edgeVertex = (deepestPointDotEdge < 0.0) ? edgeVertices[0] : edgeVertices[1];
			double d = farthestIntersectionLineCapsule(edgeVertex, -m_tn, &deepestPoint, m_penetrationPointCapsuleAxis);
			SURGSIM_ASSERT(isValid(d)) << "There must be a part of the ellipse between the triangle edge at this point";
		}

		*m_contactNormal = -m_tn;
		*m_penetrationPointCapsule = deepestPoint;
		*m_penetrationDepth = -m_tn.dot(deepestPoint - m_tv0);
		*m_penetrationPointTriangle = deepestPoint + m_tn * (*m_penetrationDepth);

		return true;
	}

	/// \param lineStart The origin of the line
	/// \param lineDir Unit directional vector of the line
	/// \param point [out] The point of intersection.
	/// \return The distance of the point of intersection from the lineStart.
	double farthestIntersectionLineCylinder(const Vector3& lineStart, const Vector3& lineDir, Vector3* point = nullptr)
	{
		if (!m_cInverseTransform.hasValue())
		{
			Vector3 j, k;
			SurgSim::Math::buildOrthonormalBasis(&m_cAxis, &j, &k);

			m_cTransform.translation() = m_cvTop;
			m_cTransform.linear().col(0) = m_cAxis;
			m_cTransform.linear().col(1) = j;
			m_cTransform.linear().col(2) = k;
			m_cInverseTransform = m_cTransform.inverse();
		}

		// Transform the problem in the cylinder space to solve the local cylinder equation y^2 + z^2 = r^2
		// Point on ellipse should be on the line, P + t.(D)
		// => Py^2 + t^2.Dy^2 + 2.Py.t.Dy + Pz^2 + t^2.Dz^2 + 2.Pz.t.Dz = r^2
		// => t^2.(Dy^2 + Dz^2) + t.(2.Py.Dy + 2.Pz.Dz) + (Py^2 + Pz^2 - r^2) = 0
		// Let a = (Dy^2 + Dz^2), b = (2.Py.Dy + 2.Pz.Dz), c = (Py^2 + Pz^2 - r^2):
		// => t^2.a + t.b + c = 0, whose solution is:
		// (-b +/- sqrt(b^2 - 4*a*c))/2*a
		auto const P = (m_cInverseTransform.getValue() * lineStart).eval();
		auto const D = (m_cInverseTransform.getValue().linear() * lineDir).eval();

		T a = D[1] * D[1] + D[2] * D[2];
		T b = static_cast<T>(2) * (P[1] * D[1] + P[2] * D[2]);
		T c = (P[1] * P[1] + P[2] * P[2] - m_cr * m_cr);

		T bb4ac = b * b - static_cast<T>(4) * a * c;

		if (bb4ac < 0.0)
		{
			// Cannot use a sqrt on a negative number. Push it to zero if it is small.
			if (bb4ac >= -Geometry::ScalarEpsilon)
			{
				bb4ac = 0.0;
			}
			else
			{
				return std::numeric_limits<T>::quiet_NaN();
			}
		}

		// We have two solutions. We want the larger value.
		double d = (-b / (static_cast<T>(2) * a)) + std::abs(std::sqrt(bb4ac) / (static_cast<T>(2) * a));
		SURGSIM_ASSERT(isValid(d));
		if (point != nullptr)
		{
			*point = lineStart + lineDir * d;
		}

		return d;
	}

	/// \param lineStart The start of the line segment
	/// \param lineDir The direction of the line segment
	/// \param point [in,out] The point which is to be clipped.
	/// \param pointOnCapsuleAxis [out] The recalculated point on the capsule axis.
	/// \return The distance of the point of intersection from the lineStart.
	double farthestIntersectionLineCapsule(const Vector3& lineStart, const Vector& lineDir,
		Vector3* point, Vector3* pointOnCapsuleAxis)
	{
		// Transform the problem in the capsule space to solve the local capsule equation:
		// case 1: x^2 + y^2 + z^2 = r^2				| x < 0
		// case 2: y^2 + z^2 = r^2						| 0 < x < length
		// case 3: (x - length)^2 + y^2 + z^2 = r^2		| x > length
		// Point should be on the line, P + t.(D)

		// case 2:
		double d = farthestIntersectionLineCylinder(lineStart, lineDir, point);
		SURGSIM_ASSERT(isValid(d));
		*point = lineStart + lineDir * d;
		auto const start = (m_cInverseTransform.getValue() * lineStart).eval();

		// case 1 and 3:
		// => ((P + t.D).x - l)^2 + (P + t.D).y^2 + (P + tD).z^2 = r^2
		// => Px^2 + t^2.Dx^2 + l^2 + 2.Px.t.Dx - 2.t.Dx.l - 2.Px.l +
		//    Py^2 + t^2.Dy^2 + 2.Py.t.Dy + Pz^2 + t^2.Dz^2 + 2.Pz.t.Dz = r^2
		// => t^2.(Dx^2 + Dy^2 + Dz^2) + t.(2.Px.Dx + 2.Py.Dy + 2.Pz.Dz - 2.Dx.l) +
		//    (Px^2 + Py^2 + Pz^2 + l^2 - 2.Px.l - r^2) = 0
		// Let a = (Dx^2 + Dy^2 + Dz^2), b = (2.Px.Dx + 2.Py.Dy + 2.Pz.Dz - 2.Dx.l),
		//     c = (Px^2 + Py^2 + Pz^2 + l^2 - 2.Px.l - r^2):
		double x = ((*point) - m_cvTop).dot(m_cAxis);
		if (x <= 0.0 || x >= m_cLength)
		{
			x = (x <= 0.0) ? 0.0 : m_cLength;

			auto const P = (m_cInverseTransform.getValue() * lineStart).eval();
			auto const D = (m_cInverseTransform.getValue().linear() * lineDir).eval();

			T a = D[0] * D[0] + D[1] * D[1] + D[2] * D[2];
			T b = static_cast<T>(2) * (P[0] * D[0] + P[1] * D[1] + P[2] * D[2] - D[0] * x);
			T c = (P[0] * P[0] + P[1] * P[1] + P[2] * P[2] + x * x - static_cast<T>(2) * P[0] * x - m_cr * m_cr);

			// => t^2.a + t.b + c = 0, whose solution is:
			// (-b +/- sqrt(b^2 - 4*a*c))/2*a
			T bb4ac = b * b - static_cast<T>(4) * a * c;

			if (bb4ac < 0.0 && bb4ac >= -Geometry::ScalarEpsilon)
			{
				bb4ac = 0.0;
			}

			// We have two solutions. We want the smaller value.
			d = (-b / (static_cast<T>(2) * a)) - std::abs(std::sqrt(bb4ac) / (static_cast<T>(2) * a));
			SURGSIM_ASSERT(isValid(d));
			*point = lineStart + lineDir * d;
		}

		*pointOnCapsuleAxis = m_cTransform * Vector3(x, 0.0, 0.0);
		return d;
	}

	/// \param segmentStart [in,out] The start of the line segment
	/// \param segmentEnd [in,out] The end of the line segment
	/// \param v The vertices of the triangle.
	/// \param planeN Normals of the triangle and each of the edge planes.
	/// \param planeD d from plane equation for the plane of the triangle and each of the edge planes.
	/// \return The index of the last plane which clips the segment passed in.
	size_t clipSegmentAgainstTriangle(Vector3* segmentStart, Vector3* segmentEnd, Vector3* v, Vector3* planeN,
		double* planeD)
	{
		double ratio, dStart, dEnd;
		size_t j = 4;
		for (size_t i = 0; i < 4; ++i)
		{
			dStart = segmentStart->dot(planeN[i]) + planeD[i];
			dEnd = segmentEnd->dot(planeN[i]) + planeD[i];

			if (dStart < -m_epsilon && dEnd > m_epsilon)
			{
				ratio = std::abs(dStart) / (std::abs(dStart) + dEnd);
				*segmentEnd = *segmentStart + (*segmentEnd - *segmentStart) * ratio;
				j = i;
			}
			else if (dStart > m_epsilon && dEnd < -m_epsilon)
			{
				ratio = dStart / (dStart + std::abs(dEnd));
				*segmentStart = *segmentStart + (*segmentEnd - *segmentStart) * ratio;
				j = i;
			}
			else if (dStart < m_epsilon && dEnd > m_epsilon)
			{
				*segmentEnd = *segmentStart;
				j = i;
			}
			else if (dStart > m_epsilon && dEnd < m_epsilon)
			{
				*segmentStart = *segmentEnd;
				j = i;
			}
		}

		SURGSIM_ASSERT(j < 4) << "The clipping should have happened at least with the triangle plane.";
		return j;
	}

	/// Triangle vertices and normal.
	Vector3 m_tv0, m_tv1, m_tv2, m_tn;
	/// Capsule ends, axis , radius and length.
	Vector3 m_cvTop, m_cvBottom, m_cAxis;
	double m_cr, m_cLength;
	/// Distance between triangle and capsule
	double m_distance;
	/// Contact info
	T* m_penetrationDepth;
	Vector3* m_penetrationPointTriangle;
	Vector3* m_penetrationPointCapsule;
	Vector3* m_contactNormal;
	Vector3* m_penetrationPointCapsuleAxis;
	/// The inverse transform of the capsule
	RigidTransform3 m_cTransform;
	SurgSim::DataStructures::OptionalValue<RigidTransform3> m_cInverseTransform;
	/// epsilon
	T m_epsilon;
};
}

template <class T, int MOpt> inline
bool calculateContactTriangleCapsule(
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& tn,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv1,
	double cr,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointTriangle,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointCapsule,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointCapsuleAxis)
{
	TriangleCapsuleContactCalculation::TriangleCapsuleContactCalculation<T, MOpt>
		calc(tv0, tv1, tv2, tn, cv0, cv1, cr, penetrationDepth, penetrationPointTriangle, penetrationPointCapsule,
			 contactNormal, penetrationPointCapsuleAxis);

	if (calc.isIntersecting())
	{
		calc.calculateContact();
		return true;
	}

	return false;
}


template <class T, int MOpt> inline
	bool calculateContactTriangleCapsule(
	const Eigen::Matrix<T, 3, 1, MOpt>& tv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv1,
	const Eigen::Matrix<T, 3, 1, MOpt>& tv2,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv0,
	const Eigen::Matrix<T, 3, 1, MOpt>& cv1,
	double cr,
	T* penetrationDepth,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint0,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPoint1,
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal,
	Eigen::Matrix<T, 3, 1, MOpt>* penetrationPointCapsuleAxis)
{
	Eigen::Matrix<T, 3, 1, MOpt> tn = (tv1 - tv0).cross(tv2 - tv0);
	Eigen::Matrix<T, 3, 1, MOpt> ca = cv1 - cv0;
	if (tn.isZero() || ca.isZero())
	{
		// Degenerate triangle/capsule passed to calculateContactTriangleTriangle
		return false;
	}
	tn.normalize();
	return calculateContactTriangleCapsule(tv0, tv1, tv2, tn, cv0, cv1, cr, penetrationDepth,
		penetrationPoint0, penetrationPoint1, contactNormal, penetrationPointCapsuleAxis);
}

} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_TRIANGLECAPSULECONTACTCALCULATION_INL_H
