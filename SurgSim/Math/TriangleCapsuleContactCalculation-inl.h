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

namespace SurgSim
{

namespace Math
{

namespace TriangleCapsuleHelper
{

/// Class used to find the intersection between a parametric line and this cylinder.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
template <class T, int MOpt>
class CylinderHelper
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> RigidTransform3;

public:
	/// \param point Any point on the cylinder axis.
	/// \param axis The axis of the cylinder of unit length
	/// \param nonAxis An unit vector that is not parallel to the axis, to build the cylinder coordinate axis.
	/// \param radius Radius of the cylinder
	CylinderHelper(const Vector3& point, const Vector3& axis, const Vector3& nonAxis, const double radius)
		: m_point(point), m_axis(axis), m_radius(radius)
	{
		RigidTransform3 transform;
		transform.translation() = m_point;
		transform.linear().col(0) = m_axis;
		transform.linear().col(2) = m_axis.cross(nonAxis).normalized();
		transform.linear().col(1) = transform.linear().col(2).cross(m_axis).normalized();
		m_inverseTransform = transform.inverse();
	}

	/// \param lineStart The origin of the line
	/// \param lineDir Unit directional vector of the line
	/// \return Parameter t of (lineStart + t*lineDir) which intersects this cylinder furthest along this line.
	double solveFarthestIntersectionWithLine(const Vector3& lineStart, const Vector& lineDir)
	{
		// Transform the problem in the cylinder space to solve the local cylinder equation y^2 + z^2 = r^2
		// Point on ellipse should be on the line, P + t.(D)
		// => Py^2 + t^2.Dy^2 + 2.Py.t.Dy + Pz^2 + t^2.Dz^2 + 2.Pz.t.Dz = r^2
		// => t^2.(Dy^2 + Dz^2) + t.(2.Py.Dy + 2.Pz.Dz) + (Py^2 + Pz^2 - r^2) = 0
		// Let a = (Dy^2 + Dz^2), b = (2.Py.Dy + 2.Pz.Dz), c = (Py^2 + Pz^2 - r^2):
		// => t^2.a + t.b + c = 0, whose solution is:
		// (-b +/- sqrt(b^2 - 4*a*c))/2*a
		auto const P = m_inverseTransform * lineStart;
		auto const D = m_inverseTransform.linear() * lineDir;

		T a = D[1] * D[1] + D[2] * D[2];
		T b = static_cast<T>(2) * (P[1] * D[1] + P[2] * D[2]);
		T c = (P[1] * P[1] + P[2] * P[2] - m_radius * m_radius);

		// We have two solutions. We want the larger value.
		return (-b / (static_cast<T>(2) * a))
			+ std::abs(std::sqrt(b * b - static_cast<T>(4) * a * c) / (static_cast<T>(2) * a));
	}

private:
	/// A point on the cylinder
	Vector3 m_point;
	/// Axis of the cylinder
	Vector3 m_axis;
	/// Radius of the cylinder
	double m_radius;
	/// Transform from world coordinates to cylinder local coordinates
	RigidTransform3 m_inverseTransform;
};

/// Class used to find the point on (positive X side of) ellipse with the given slope.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
template <class T, int MOpt>
class EllipseHelper
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> RigidTransform3;

public:
	/// \param center Center of the ellipsis.
	/// \param majorAxis, minorAxis The major/minor axes of the ellipse, both of unit length
	/// \param majorRadius, minorRadius Major/minor radii of the ellipse
	EllipseHelper(const Vector3& center, const Vector3& majorAxis, const Vector3& minorAxis, const double majorRadius,
		const double minorRadius) : a(majorRadius), b(minorRadius)
	{
		m_transform.translation() = center;
		m_transform.linear().col(0) = majorAxis;
		m_transform.linear().col(1) = minorAxis;
		m_transform.linear().col(2) = majorAxis.cross(minorAxis);
		m_inverseTransform = m_transform.inverse();
	}

	/// \param tangent The given tangent to this ellipse, whose correcponding point is to be found
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
		// This slope is equal to the slope of the localTriangleEdge. So, we can solve for x and y.
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
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal)
{
	typedef Eigen::Matrix<T, 3, 1, MOpt> Vector3;
	typedef Eigen::Transform<T, 3, Eigen::Isometry> RigidTransform3;
	static const T EPSILON = static_cast<T>(Geometry::DistanceEpsilon);

	double distance =
		distanceSegmentTriangle(cv0, cv1, tv0, tv1, tv2, tn, penetrationPointCapsule, penetrationPointTriangle);

	// Check if the triangle and capsule intersect.
	if (distance >= (cr - EPSILON))
	{
		// Distance between the triangle and the capsule's axis is greater than the radius of the capsule.
		// Therefore, there is no intersection.
		return false;
	}

	// Rest of the below cases have an intersection.
	if (distance > EPSILON)
	{
		// The axis of the capsule does not intersect with the triangle, so a correction of (cr - distance) along the
		// closest points between the two, is the shortest contact correction possible.
		*contactNormal = *penetrationPointTriangle - *penetrationPointCapsule;
		contactNormal->normalize();
		*penetrationDepth = cr - distance;
		*penetrationPointCapsule += (*contactNormal * cr);
		return true;
	}

	// The axis of the capsule intersects with the triangle face. Contact is corrected by moving along the
	// triangle's normal.

	// One end of the capsule axis is over the triangle plane and the other is under. The capsule axis is calculated
	// such that it points from above the triangle to below the triangle.
	Vector3 capsuleTop = cv0;
	Vector3 capsuleBottom = cv1;
	Vector3 capsuleAxis = (capsuleBottom - capsuleTop).normalized();
	if (capsuleAxis.dot(tn) > 0.0)
	{
		capsuleTop = cv1;
		capsuleBottom = cv0;
		capsuleAxis = -capsuleAxis;
	}

	// If the capsule axis is perpendicular to the triangle, then the deepest penetration point on the capsule axis
	// is capsuleBottom
	if (std::abs(std::abs(capsuleAxis.dot(tn)) - 1.0) < EPSILON)
	{
		*contactNormal = -tn;
		*penetrationDepth = cr + (*penetrationPointCapsule - capsuleBottom).dot(tn);
		*penetrationPointCapsule = capsuleBottom - tn * cr;
		return true;
	}

	// At this point, the capsule axis intersects with one of the planes at the triangle edges (containing the
	// triangle normal). This point of intersection is the deepestPoint on the capsule axis.
	Vector3 deepestPoint;

	const Vector3 v[3] = {tv0, tv1, tv2};
	Vector3 triangleEdge;
	Vector3 planeNormal;
	double planeD;
	Vector3 edgeVertices[2];

	bool axisIntersectsPlane = false;
	for (size_t i = 0; i < 3; ++i)
	{
		triangleEdge = v[(i + 1) % 3] - v[i];
		planeNormal = tn.cross(triangleEdge);
		planeNormal.normalize();
		planeD = v[i].dot(planeNormal);

		double capsuleBottomD = capsuleBottom.dot(planeNormal) - planeD;
		if (capsuleBottomD < 0.0)
		{
			double capsuleTopD = capsuleTop.dot(planeNormal) - planeD;
			SURGSIM_ASSERT(capsuleTopD > 0.0);
			double ratio = capsuleTopD / (capsuleTopD + std::abs(capsuleBottomD));

			deepestPoint = capsuleTop + (capsuleBottom - capsuleTop) * ratio;

			edgeVertices[0] = v[i];
			edgeVertices[1] = v[(i + 1) % 3];

			axisIntersectsPlane = true;
			break;
		}
	}

	if (!axisIntersectsPlane)
	{
		*contactNormal = -tn;
		*penetrationDepth = cr + (*penetrationPointCapsule - capsuleBottom).dot(tn);
		*penetrationPointCapsule = capsuleBottom - tn * cr;
		*penetrationPointTriangle = *penetrationPointCapsule + tn * *penetrationDepth;
		return true;
	}

	// The capsule axis intersects the plane (planeNormal, planeD). First, the capsule is treated as a cylinder such
	// its intersection with this plane is given by an ellipse. The deepest point on this ellipse along -tn and bounded
	// by vectors (edge[0] -> edge[0] - tn) and (edge[1] -> edge[1] - tn) is found.
	Vector3 center = deepestPoint;
	triangleEdge.normalize();
	Vector3 majorAxis = triangleEdge * (triangleEdge.dot(capsuleAxis)) + (-tn) * ((-tn).dot(capsuleAxis));
	majorAxis.normalize();

	// CylinderHelper used to find the point of (farthest) intersection along the line and the cylinder.
	TriangleCapsuleHelper::CylinderHelper<T, MOpt> cylinderHelper(capsuleTop, capsuleAxis, -tn, cr);

	double majorRadius = cylinderHelper.solveFarthestIntersectionWithLine(center, majorAxis);
	deepestPoint = center + majorAxis * majorRadius;
	
	if (std::abs(majorAxis.dot(triangleEdge)) > EPSILON)
	{
		// majorApex is not the deepest point because the ellipse is angled. The deepest point is between majorApex and
		// minorApex on the circumference of the ellipse, and the tangent at that point is parallel to the triangleEdge.
		auto minorAxis = planeNormal.cross(majorAxis);
		double minorRadius = cylinderHelper.solveFarthestIntersectionWithLine(center, minorAxis);
		
		TriangleCapsuleHelper::EllipseHelper<T, MOpt>
			ellipseHelper(center, majorAxis, minorAxis, majorRadius, minorRadius);
		deepestPoint = ellipseHelper.pointWithTangent(triangleEdge);
	}

	// Project deepestPoint on the triangle edge to make sure it is within the edge.
	auto edgeLength = (edgeVertices[1] - edgeVertices[0]).norm();
	double deepestPointDotEdge = triangleEdge.dot(deepestPoint - edgeVertices[0]);
	if (deepestPointDotEdge <= -EPSILON || deepestPointDotEdge >= edgeLength + EPSILON)
	{
		// In this case, the intersection of the cylinder with the triangle edge plane gives an ellipse
		// that is close to a triangle corner and the deepest penetration point on the ellipse is
		// actually outside the triangle.
		// Solution: find the deepest point on the ellipse which projection is still on the triangle.
		// For that: Find the corner edgeVertex of the triangle closest to deepestPointDotEdge
		// Find the deepest penetration point verifying P - tn*t and the cylinder equation.

		// The triangle point to consider is edgeVertices[0] or edgeVertices[1].
		Vector3 edgeVertex = (deepestPointDotEdge < 0.0) ? edgeVertices[0] : edgeVertices[1];
		double t = cylinderHelper.solveFarthestIntersectionWithLine(edgeVertex, -tn);
		deepestPoint = edgeVertex + t * (-tn);
	}

	Vector3 pointOnCapsuleAxis;
	// Now, clip (center -> deepestPoint) to be on the surface of the capsule. This is done by finding its length.
	double distanceFromCapsule = SurgSim::Math::distancePointSegment(deepestPoint, cv0, cv1, &pointOnCapsuleAxis);
	if (std::abs(distanceFromCapsule - cr) > EPSILON)
	{
		Vector3d direction = (deepestPoint - center).normalized();
		deepestPoint = center + direction * cr;
	}

	*contactNormal = -tn;
	*penetrationPointCapsule = deepestPoint;
	*penetrationDepth = -tn.dot(deepestPoint - tv0);
	*penetrationPointTriangle = deepestPoint + tn * (*penetrationDepth);

	return true;
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
	Eigen::Matrix<T, 3, 1, MOpt>* contactNormal)
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
		penetrationPoint0, penetrationPoint1, contactNormal);
}

} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_TRIANGLECAPSULECONTACTCALCULATION_INL_H