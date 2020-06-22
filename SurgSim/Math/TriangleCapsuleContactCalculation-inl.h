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

/// Find the point on (positive X side of) ellipse parallel to the given tangent.
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param tangent The given tangent to this ellipse, whose corresponding point is to be found
/// \param center Center of the ellipse.
/// \param majorAxis, minorAxis The major/minor axes of the ellipse, both of unit length
/// \param majorRadius, minorRadius Major/minor radii of the ellipse
/// \note majorAxis and minorAxis are assumed to be orthogonal to each other.
/// \return The point on the ellipse (in positive x direction) which has the given tangent.
template <class T, int MOpt>
Eigen::Matrix<T, 3, 1, MOpt> pointWithTangentOnEllipse(const Eigen::Matrix<T, 3, 1, MOpt>& center,
													   const Eigen::Matrix<T, 3, 1, MOpt>& majorAxis,
													   const Eigen::Matrix<T, 3, 1, MOpt>& minorAxis,
													   const double majorRadius, const double minorRadius,
													   const Eigen::Matrix<T, 3, 1, MOpt>& tangent)
{
	Eigen::Transform<T, 3, Eigen::Isometry> transform;
	transform.translation() = center;
	transform.linear().col(0) = majorAxis;
	transform.linear().col(1) = minorAxis;
	transform.linear().col(2) = majorAxis.cross(minorAxis);

	// tangent in local coordinates.
	Eigen::Matrix<T, 3, 1, MOpt> localTangent = transform.inverse().linear() * tangent;

	// Slope of this tangent
	T m = localTangent[1] / localTangent[0];

	// Ellipse equation: x*x/a*a + y*y/b*b = 1
	// Rewriting ellipse equation in the form, y = f(x): y = sqrt(a*a - x*x) * b / a
	// Slope of ellipse: y' = -x*b*b/a*a*y
	// This slope is equal to the slope of the localTangent. So, we can solve for x and y.
	T x = std::sqrt((m * m * majorRadius * majorRadius * majorRadius * majorRadius) /
					(minorRadius * minorRadius + m * m * majorRadius * majorRadius));
	T y = (minorRadius / majorRadius) *
		std::sqrt(majorRadius * majorRadius - x * x) * ((m > 0.0) ? -1.0 : 1.0);

	// Transforming this point into world coordinates.
	return transform * Eigen::Matrix<T, 3, 1, MOpt>(x, y, static_cast<T>(0));
}

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
	TriangleCapsuleContactCalculation(
		const Vector3& tv0, const Vector3& tv1, const Vector3& tv2,
		const Vector3& tn,
		const Vector3& cv0, const Vector3& cv1,
		double cr)
		: m_tv0(tv0), m_tv1(tv1), m_tv2(tv2), m_tn(tn),
		  m_cvTop(cv0), m_cvBottom(cv1), m_cr(cr)
	{
		m_epsilon = static_cast<T>(Geometry::DistanceEpsilon);
		m_distance = distanceSegmentTriangle(cv0, cv1, m_tv0, m_tv1, m_tv2, m_tn,
			&m_penetrationPointCapsuleAxis, &m_penetrationPointTriangle);
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
	/// \param [out] penetrationDepth The depth of penetration.
	/// \param [out] penetrationPointTriangle The contact point on triangle.
	/// \param [out] penetrationPointCapsule The contact point on capsule.
	/// \param [out] contactNormal The contact normal that points from capsule to triangle.
	/// \param [out] penetrationPointCapsuleAxis The point on the capsule axis closest to the triangle.
	void calculateContact(T* penetrationDepth, Vector3* penetrationPointTriangle, Vector3* penetrationPointCapsule,
		Vector3* contactNormal, Vector3* penetrationPointCapsuleAxis)
	{
		if (isIntersecting())
		{
			bool result =
				axisAwayFromTriangle() ||
				axisPerpendicularToTriangle() ||
				axisTouchingTriangle() ||
				axisThroughTriangle();

			SURGSIM_ASSERT(result) << __func__ << " " << __LINE__ << "At this point, there must be an intersection." <<
				std::setprecision(16) << "\nm_tv0: " << m_tv0.transpose() <<
				"\nm_tv1: " << m_tv1.transpose() << "\nm_tv2: " << m_tv2.transpose() << "\nm_cr: " << m_cr <<
				"\nm_cvTop: " << m_cvTop.transpose() << "\nm_cvBottom: " << m_cvBottom.transpose();

			*penetrationDepth = m_penetrationDepth;
			*penetrationPointTriangle = m_penetrationPointTriangle;
			*penetrationPointCapsule = m_penetrationPointCapsule;
			*contactNormal = m_contactNormal;
			*penetrationPointCapsuleAxis = m_penetrationPointCapsuleAxis;
		}
	}

	void toBytes(SurgSim::Math::Vector3d in, std::vector<uint8_t>* result)
	{
		auto ptr = reinterpret_cast<uint8_t*>(in.data());

		auto bytes = in.size() * sizeof(double);

		for (int i = 0; i < bytes; ++i)	result->push_back(ptr[i]);
	}
	size_t fromBytes(const std::vector<uint8_t>& bytes, SurgSim::Math::Vector3d* out, size_t start = 0)
	{
		auto ptr = reinterpret_cast<uint8_t*>(out->data());
		auto count = sizeof(double) * out->size();
		memcpy(ptr, bytes.data() + start, count);
		return count;
	}

	void toBytes(double d, std::vector<uint8_t>* result)
	{
		auto ptr = reinterpret_cast<uint8_t*>(&d);
		auto bytes = sizeof(double);
		for (int i = 0; i < bytes; ++i) result->push_back(ptr[i]);
	}

	size_t fromBytes(const std::vector<uint8_t>& bytes, double* out, size_t start = 0)
	{
		auto ptr = reinterpret_cast<uint8_t*>(out);
		memcpy(ptr, bytes.data() + start, sizeof(double));
		return sizeof(double);
	}

	void toFile(const std::string& filename)
	{
		std::vector<uint8_t> bytes;
		toBytes(m_tv0, &bytes);
		toBytes(m_tv1, &bytes);
		toBytes(m_tv2, &bytes);
		toBytes(m_tn, &bytes);
		toBytes(m_cvTop, &bytes);
		toBytes(m_cvBottom, &bytes);
		toBytes(m_cr, &bytes);

		std::ofstream out(filename, std::ios_base::binary);
		SURGSIM_ASSERT(out.is_open());
		
		for (auto b : bytes) out << b;
	}

	void fromFile(const std::string& filename)
	{
		// 6 Vector 3 doubles each + 1 double = 19 doubles
		size_t size = sizeof(double) * 19;

		std::ifstream in(filename,  std::ios_base::binary);
		SURGSIM_ASSERT(in.is_open());
	
		std::vector<uint8_t> bytes(std::istreambuf_iterator<char>(in), {});

		size_t start = 0;

		start += fromBytes(bytes, &m_tv0, start);
		start += fromBytes(bytes, &m_tv1, start);
		start += fromBytes(bytes, &m_tv2, start);
		start += fromBytes(bytes, &m_tn, start);
		start += fromBytes(bytes, &m_cvTop, start);
		start += fromBytes(bytes, &m_cvBottom, start);
		start += fromBytes(bytes, &m_cr, start);
	
		SURGSIM_ASSERT(start == size);
	}

	/// This function handles the contact data calculation for the case where there is an intersection between the
	/// capsule and the triangle, but the capsule axis does not intersect the triangle.
	/// \return True, if the axis of the capsule is away from the triangle.
	/// \note This function presupposes that isIntersecting() returned true.
	bool axisAwayFromTriangle()
	{
		if (m_distance > m_epsilon)
		{
			// TODO(ryanbeasley): Verify that we want contact normals not necessarily aligned with the triangle normal.
			m_contactNormal = m_penetrationPointTriangle - m_penetrationPointCapsuleAxis;
			m_contactNormal.normalize();
			m_penetrationPointCapsule = m_penetrationPointCapsuleAxis + (m_contactNormal * m_cr);
			m_penetrationDepth = m_cr - m_distance;
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
			m_penetrationPointCapsule = m_cvBottom - m_tn * m_cr;
			m_penetrationPointCapsuleAxis = m_cvBottom;
			m_contactNormal = -m_tn;
			m_penetrationDepth = (m_tv0 - m_penetrationPointCapsule).dot(m_tn);
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
		// TODO(ryanbeasley): This function should be combined with axisThroughTriangle; both use the intersection of
		// the capsule with the prism, but with different assumptions.
		if (m_penetrationPointCapsuleAxis.isApprox(m_cvTop, m_epsilon) ||
			m_penetrationPointCapsuleAxis.isApprox(m_cvBottom, m_epsilon) ||
			isPointOnTriangleEdge(m_penetrationPointTriangle, m_tv0, m_tv1, m_tv2))
		{
			m_contactNormal = -m_tn;

			auto projectionCvBottom = (m_cvBottom + m_tn * (m_tv0 - m_cvBottom).dot(m_tn)).eval();
			if (SurgSim::Math::isPointInsideTriangle(projectionCvBottom, m_tv0, m_tv1, m_tv2, m_tn))
			{
				m_penetrationPointCapsule = m_cvBottom - m_tn * m_cr;
				m_penetrationPointCapsuleAxis = m_cvBottom;
			}
			else
			{
				Vector3 v[3] = { m_tv0, m_tv1, m_tv2 };
				Vector3 planeN[4] = { m_tn, (-m_tn.cross(m_tv1 - m_tv0)).normalized(),
					(-m_tn.cross(m_tv2 - m_tv1)).normalized(), (-m_tn.cross(m_tv0 - m_tv2)).normalized() };
				double planeD[4] = { -m_tv0.dot(planeN[0]), -m_tv0.dot(planeN[1]), -m_tv1.dot(planeN[2]),
					-m_tv2.dot(planeN[3]) };

				if (!m_cInverseTransform.hasValue())
				{
					Vector3 j, k;
					SurgSim::Math::buildOrthonormalBasis(&m_cAxis, &j, &k);

					m_cTransform.translation() = m_cvTop;
					m_cTransform.linear().col(0) = m_cAxis;
					m_cTransform.linear().col(1) = j;
					m_cTransform.linear().col(2) = k;
					m_cInverseTransform = m_cTransform.inverse();

					SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("TriangleCapsuleContactCalculation")) <<
						__func__ << " " << __LINE__ << std::setprecision(12) <<
						"  m_cInverseTransform optional was empty." <<
						"\nm_cAxis: " << m_cAxis.transpose() <<
						"\nm_cTransform.linear():\n" << m_cTransform.linear() <<
						"\nm_cTransform.translation(): " << m_cTransform.translation().transpose() <<
						"\nm_cInverseTransform.linear():\n" << m_cInverseTransform.getValue().linear() <<
						"\nm_cInverseTransform.translation(): " << m_cInverseTransform.getValue().translation().transpose();
				}
				Vector3 closestPointSegment = m_penetrationPointCapsuleAxis;
				axisTouchingTriangleWithBottomOutside(m_cvTop, m_cvBottom, projectionCvBottom, closestPointSegment,
					m_cAxis, m_cLength, m_cr, v, planeN, planeD, m_cTransform, m_cInverseTransform.getValue(),
					&m_penetrationPointCapsuleAxis, &m_penetrationPointCapsule);
			}
			m_penetrationDepth = (m_tv0 - m_penetrationPointCapsule).dot(m_tn);
			m_penetrationPointTriangle = m_penetrationPointCapsule + m_tn * m_penetrationDepth;
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
			m_contactNormal = -m_tn;
			m_penetrationPointCapsule = m_cvBottom - m_tn * m_cr;
			m_penetrationDepth = (m_tv0 - m_penetrationPointCapsule).dot(m_tn);
			m_penetrationPointCapsuleAxis = m_cvBottom;
			m_penetrationPointTriangle = m_penetrationPointCapsule + m_tn * m_penetrationDepth;
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
		if (!m_cInverseTransform.hasValue())
		{
			Vector3 j, k;
			SurgSim::Math::buildOrthonormalBasis(&m_cAxis, &j, &k);

			m_cTransform.translation() = m_cvTop;
			m_cTransform.linear().col(0) = m_cAxis;
			m_cTransform.linear().col(1) = j;
			m_cTransform.linear().col(2) = k;
			m_cInverseTransform = m_cTransform.inverse();

			SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("TriangleCapsuleContactCalculation")) <<
				__func__ << " " << __LINE__ << std::setprecision(12) <<
				"  m_cInverseTransform optional was empty." <<
				"\nm_cTransform.linear():\n" << m_cTransform.linear() <<
				"\nm_cTransform.translation(): " << m_cTransform.translation().transpose() <<
				"\nm_cInverseTransform.linear():\n" << m_cInverseTransform.getValue().linear() <<
				"\nm_cInverseTransform.translation(): " << m_cInverseTransform.getValue().translation().transpose();
		}
		double majorRadius = farthestIntersectionLineCylinder(center, majorAxis, m_cr, m_cInverseTransform.getValue(),
			&deepestPoint);
		SURGSIM_ASSERT(isValid(majorRadius)) << __func__ << " " << __LINE__<<
			"The major radius of the ellipse should be a valid number." << std::setprecision(16) <<
			"\nv[0]: " << v[0].transpose() << "\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() <<
			"\nm_cr: " << m_cr << "\nm_cvTop: " << m_cvTop.transpose() << "\nm_cvBottom: " << m_cvBottom.transpose(); 

		if (std::abs(majorAxis.dot(triangleEdge)) > m_epsilon)
		{
			// majorApex is not the deepest point because the ellipse is angled. The deepest point is between majorApex
			// and minorApex on the circumference of the ellipse, and the tangent at that point is parallel to the
			// triangleEdge.
			auto minorAxis = planeN[j].cross(majorAxis);
			double minorRadius =
				farthestIntersectionLineCylinder(center, minorAxis, m_cr, m_cInverseTransform.getValue());
			SURGSIM_ASSERT(isValid(minorRadius)) << __func__ << " " << __LINE__ <<
				"The minor radius of the ellipse should be a valid number." << std::setprecision(16) <<
				"\nv[0]: " << v[0].transpose() << "\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() <<
				"\nm_cr: " << m_cr << "\nm_cvTop: " << m_cvTop.transpose() <<
				"\nm_cvBottom: " << m_cvBottom.transpose();

			deepestPoint = pointWithTangentOnEllipse(center, majorAxis, minorAxis, majorRadius, minorRadius,
				triangleEdge);
			Vector3 result;
			if (std::abs(distancePointSegment(deepestPoint, m_cvTop, m_cvBottom, &result) - m_cr) > m_epsilon)
			{
				// The deepest point is not on the capsule, which means that the capsule end (the sphere) is
				// intersecting the triangle edge plane (planeN[j], planeD[j]). The intersection between them is a
				// circle. Define a 2D co-ordinate system with the origin at edgeVertices[0], the x-axis as
				// triangleEdge, and the y-axis as tn. Transforming the circle to this 2D co-ordinate system, creates a
				// circle of radius, r, with its center at x, y. Now the deepest point on this circle is (x, y - r).
				Vector3 origin = edgeVertices[0], xAxis = triangleEdge, yAxis = m_tn, zAxis = planeN[j];

				double sphereCenterToXYPlane = (m_cvBottom - origin).dot(zAxis);
				SURGSIM_ASSERT(m_cr + m_epsilon >= sphereCenterToXYPlane) << __func__ << " " << __LINE__ <<
					" There is an intersection between a capsule and a triangle, and the capsule axis may go " <<
					"through the triangle.\nThe segment clips against one of the planes of the triangle, but " <<
					"not the plane containing all three vertices.\nThe deepest point is not on the cylinder, " <<
					"so it should be on the sphere at the end of the capsule, but the sphere center is too far " <<
					"from the triangle edge plane." << std::setprecision(16) << "\nv[0]: " << v[0].transpose() <<
					"\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() << "\nm_cr: " << m_cr <<
					"\nm_cvTop: " << m_cvTop.transpose() << "\nm_cvBottom: " << m_cvBottom.transpose();

				double circleRadius = std::sqrt(m_cr * m_cr - sphereCenterToXYPlane * sphereCenterToXYPlane);
				SURGSIM_ASSERT(isValid(circleRadius)) << __func__ << " " << __LINE__ <<
					" There is an intersection between a capsule and a triangle, and the capsule axis may go " <<
					"through the triangle.\nThe segment clips against one of the planes of the triangle, but " <<
					"not the plane containing all three vertices.\nThe deepest point is not on the cylinder, " <<
					"so it should be on the sphere at the end of the capsule, but the " <<
					"radius of the circle of intersection between the sphere and the plane is invalid." <<
					std::setprecision(16) << "\nv[0]: " << v[0].transpose() <<
					"\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() << "\nm_cr: " << m_cr <<
					"\nm_cvTop: " << m_cvTop.transpose() << "\nm_cvBottom: " << m_cvBottom.transpose();
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
			double d = farthestIntersectionLineCapsule(m_cvTop, m_cAxis, m_cLength, m_cr, edgeVertex, -m_tn,
				m_cTransform, m_cInverseTransform.getValue(), &deepestPoint, &m_penetrationPointCapsuleAxis);
			SURGSIM_ASSERT(isValid(d)) << __func__ << " " << __LINE__ <<
				"There must be a part of the ellipse between the triangle edge at this point." <<
				std::setprecision(16) << "\nv[0]: " << v[0].transpose() <<
				"\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() << "\nm_cr: " << m_cr <<
				"\nm_cvTop: " << m_cvTop.transpose() << "\nm_cvBottom: " << m_cvBottom.transpose();
		}

		m_contactNormal = -m_tn;
		m_penetrationPointCapsule = deepestPoint;
		m_penetrationDepth = -m_tn.dot(deepestPoint - m_tv0);
		m_penetrationPointTriangle = deepestPoint + m_tn * (m_penetrationDepth);

		return true;
	}

	/// \param lineStart The origin of the line
	/// \param lineDir Unit directional vector of the line
	/// \param cr The capsule radius.
	/// \param cInverseTransform The rigid transform to cylinder coordinates.
	/// \param point [out] The point of intersection.
	/// \return The distance of the point of intersection from the lineStart.
	static T farthestIntersectionLineCylinder(const Vector3& lineStart, const Vector3& lineDir, T cr,
		const RigidTransform3& cInverseTransform, Vector3* point = nullptr)
	{
		// Transform the problem in the cylinder space to solve the local cylinder equation y^2 + z^2 = r^2
		// Point on ellipse should be on the line, P + t.(D)
		// => Py^2 + t^2.Dy^2 + 2.Py.t.Dy + Pz^2 + t^2.Dz^2 + 2.Pz.t.Dz = r^2
		// => t^2.(Dy^2 + Dz^2) + t.(2.Py.Dy + 2.Pz.Dz) + (Py^2 + Pz^2 - r^2) = 0
		// Let a = (Dy^2 + Dz^2), b = (2.Py.Dy + 2.Pz.Dz), c = (Py^2 + Pz^2 - r^2):
		// => t^2.a + t.b + c = 0, whose solution is:
		// (-b +/- sqrt(b^2 - 4*a*c))/2*a
		auto const P = (cInverseTransform * lineStart).eval();
		auto const D = (cInverseTransform.linear() * lineDir).eval();

		T a = D[1] * D[1] + D[2] * D[2];
		T b = static_cast<T>(2) * (P[1] * D[1] + P[2] * D[2]);
		T c = (P[1] * P[1] + P[2] * P[2] - cr * cr);

		T discriminant = b * b - static_cast<T>(4) * a * c;

		if (discriminant < 0.0)
		{
			// Cannot use a sqrt on a negative number. Push it to zero if it is small.
			if (discriminant >= -Geometry::ScalarEpsilon)
			{
				discriminant = 0.0;
			}
			else
			{
				return std::numeric_limits<T>::quiet_NaN();
			}
		}

		// We have two solutions. We want the larger value.
		double d = (-b / (static_cast<T>(2) * a)) + std::abs(std::sqrt(discriminant) / (static_cast<T>(2) * a));
		SURGSIM_ASSERT(isValid(d));
		if (point != nullptr)
		{
			*point = lineStart + lineDir * d;
		}

		return d;
	}

	/// \param cvTop The top endpoint of the capsule.
	/// \param cAxis The capsule axis.
	/// \param cLength The capsule length.
	/// \param cr The capsule radius.
	/// \param lineStart The start of the line segment
	/// \param lineDir The direction of the line segment
	/// \param cTransform The rigid transform from cylinder coordinates.
	/// \praam cInversTransform The rigid transform to cylinder coordinates.
	/// \param point [out] The point which is to be clipped.
	/// \param pointOnCapsuleAxis [out] The recalculated point on the capsule axis.
	/// \return The distance of the point of intersection from the lineStart.
	/// \note Asserts when there is no intersection.
	static T farthestIntersectionLineCapsule(const Vector3& cvTop, const Vector3& cAxis, T cLength, T cr,
		const Vector3& lineStart, const Vector& lineDir,
		const RigidTransform3& cTransform, const RigidTransform3& cInverseTransform,
		Vector3* point, Vector3* pointOnCapsuleAxis)
	{
		// Transform the problem into the capsule space to solve the local capsule equation. The capsule coordinate
		// system has its origin on one the capsule ends (cvTop), the x axis is along the capsule axis (cAxis), and
		// the y and z axes are any orthogonal vectors to the capsule axis. The equation of the capsule can be written
		// as the following:
		// x^2 + y^2 + z^2 = r^2				| x < 0				------ [1]
		// y^2 + z^2 = r^2						| 0 < x < length	------ [2]
		// (x - length)^2 + y^2 + z^2 = r^2		| x > length		------ [3]
		// Point should be on the line, P + t.(D)

		// First find the intersection with an infinite cylinder centered around the capsule axis.
		double d = farthestIntersectionLineCylinder(lineStart, lineDir, cr, cInverseTransform, point);
		SURGSIM_ASSERT(isValid(d));

		// When x <0 or x > length, the equation of the capsule is that of the sphere (from equations [1] and [3]). So,
		// the intersection between the line (P + t.D) and the sphere is calculated as below. Here l is the length of
		// the capsule.
		// => ((P + t.D).x - l)^2 + (P + t.D).y^2 + (P + t.D).z^2 = r^2
		// => Px^2 + t^2.Dx^2 + l^2 + 2.Px.t.Dx - 2.t.Dx.l - 2.Px.l +
		//    Py^2 + t^2.Dy^2 + 2.Py.t.Dy + Pz^2 + t^2.Dz^2 + 2.Pz.t.Dz = r^2
		// => t^2.(Dx^2 + Dy^2 + Dz^2) + t.(2.Px.Dx + 2.Py.Dy + 2.Pz.Dz - 2.Dx.l) +
		//    (Px^2 + Py^2 + Pz^2 + l^2 - 2.Px.l - r^2) = 0
		// Let a = (Dx^2 + Dy^2 + Dz^2),
		//     b = (2.Px.Dx + 2.Py.Dy + 2.Pz.Dz - 2.Dx.l),
		//     c = (Px^2 + Py^2 + Pz^2 + l^2 - 2.Px.l - r^2):
		double x = ((*point) - cvTop).dot(cAxis);
		if (x <= 0.0 || x >= cLength)
		{
			x = (x <= 0.0) ? 0.0 : cLength;

			auto const P = (cInverseTransform * lineStart).eval();
			auto const D = (cInverseTransform.linear() * lineDir).eval();

			//T a = D[0] * D[0] + D[1] * D[1] + D[2] * D[2];  lineDir is normalized so this is 1.0
			T b = static_cast<T>(2) * (P[0] * D[0] + P[1] * D[1] + P[2] * D[2] - D[0] * x);
			T c = (P[0] * P[0] + P[1] * P[1] + P[2] * P[2] + x * x - static_cast<T>(2) * P[0] * x - cr * cr);

			// => t^2.a + t.b + c = 0, whose solution is:
			// (-b +/- sqrt(b^2 - 4*a*c))/2*a ... where a = 1
			T discriminant = b * b - static_cast<T>(4) * c;

			if (discriminant < 0.0 && discriminant >= -Geometry::ScalarEpsilon)
			{
				discriminant = 0.0;
			}

			// We have two solutions. We want the positive value.
			d = (-b + std::abs(std::sqrt(discriminant))) * static_cast<T>(0.5);
			SURGSIM_ASSERT(isValid(d));
			*point = lineStart + lineDir * d;
		}

		*pointOnCapsuleAxis = cTransform * Vector3(x, 0.0, 0.0);
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

			if (dStart < 0.0 && dEnd > 0.0)
			{
				ratio = std::abs(dStart) / (std::abs(dStart) + dEnd);
				*segmentEnd = *segmentStart + (*segmentEnd - *segmentStart) * ratio;
				j = i;
			}
			else if (dStart > 0.0 && dEnd < 0.0)
			{
				ratio = dStart / (dStart + std::abs(dEnd));
				*segmentStart = *segmentStart + (*segmentEnd - *segmentStart) * ratio;
				j = i;
			}
		}
		SURGSIM_ASSERT(j < 4) << "The clipping should have happened at least with the triangle plane.";
		return j;
	}

	/// This function calculates the deepest penetration point on the capsule, assuming that the axis touches the
	/// triangle, and bottom endpoint projects outside of the triangle.
	/// \param top The start of the line segment.
	/// \param bottom The end of the line segment.
	/// \param projectionBottom The projection of the bottom endpoint into the triangle plane.
	/// \param closestPointSegment The closest point on the segment to the triangle.
	/// \param cAxis The capsule axis.
	/// \param cLength The capsule length.
	/// \param cr The capsule radius.
	/// \param v The vertices of the triangle.
	/// \param planeN Normals of the triangle and each of the edge planes.
	/// \param planeD d from plane equation for the plane of the triangle and each of the edge planes.
	/// \param cTransform The rigid transform from cylinder coordinates.
	/// \param cInverseTransform The rigid transform to cylinder coordinates.
	/// \param [out] penetrationPointCapsuleAxis The point on the capsule axis used for the contact.
	/// \param [out] penetrationPointCapsule The point on the capsule used for the contact.
	/// \exception Asserts if does not find a contact.
	static void axisTouchingTriangleWithBottomOutside(const Vector3& top, const Vector3& bottom,
		const Vector3& projectionBottom, const Vector3& closestPointSegment, const Vector3& cAxis, T cLength, T cr,
		Vector3* v, Vector3* planeN, T* planeD,
		const RigidTransform3& cTransform, const RigidTransform3& cInverseTransform,
		Vector3* penetrationPointCapsuleAxis, Vector3* penetrationPointCapsule)
	{
		T bottomDistance[3]; // Distance from bottom to each edge plane.
		for (int i = 0; i < 3; ++i)
		{
			bottomDistance[i] = bottom.dot(planeN[i + 1]) + planeD[i + 1];
		}

		if (bottomDistance[0] <= static_cast<T>(0) && bottomDistance[1] <= static_cast<T>(0) &&
			bottomDistance[2] <= static_cast<T>(0))
		{
			*penetrationPointCapsuleAxis = bottom;
			*penetrationPointCapsule = bottom - planeN[0] * cr;
			return;
		}

		const T crSquared = cr * cr;
		for (int i = 0; i < 3; ++i)
		{
			if (bottomDistance[i] > 0 && bottomDistance[i] <= cr)
			{ // bottom is within radius of an edge.
				T depth = std::sqrt(crSquared - bottomDistance[i] * bottomDistance[i]);
				Vector3 deepestPoint = bottom - bottomDistance[i] * planeN[i + 1] - planeN[0] * depth;
				if (isPointInsideTriangle(deepestPoint, v[0], v[1], v[2], planeN[0]))
				{
					// If the x-coordinate is less than the endpoint, then a point on the axis will contact deeper.
					if ((cInverseTransform * deepestPoint)[0] >= cLength)
					{
						*penetrationPointCapsuleAxis = bottom;
						*penetrationPointCapsule = deepestPoint;
						return;
					}
				}
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			if ((projectionBottom - v[i]).squaredNorm() <= crSquared)
			{ // bottom is within radius of a vertex.
				Vector3 deepestPoint;
				farthestIntersectionLineSphere(v[i], v[i] - planeN[0], bottom, cr, &deepestPoint);
				if ((cInverseTransform * deepestPoint)[0] >= cLength)
				{
					*penetrationPointCapsuleAxis = bottom;
					*penetrationPointCapsule = deepestPoint;
					return;
				}
			}
		}

		if (cLength < Geometry::DistanceEpsilon)
		{
			*penetrationPointCapsuleAxis = closestPointSegment;
			*penetrationPointCapsule = closestPointSegment - cr * planeN[0];
			return;
		}

		for (int i = 0; i < 3; ++i)
		{
			if ((cInverseTransform.linear() * planeN[i + 1])[0] > Geometry::DistanceEpsilon)
			{ // The cylinder moves in the direction of the plane normal. First, the capsule is treated as a cylinder.
				Vector3 center; // Intersection of the line along the axis with the plane.
				T dist = distanceLinePlane(top, bottom, planeN[i + 1], planeD[i + 1], &center);
				Vector3 edgeVertices[2] = { v[i], v[(i + 1) % 3] };
				Vector3 triangleEdge = (edgeVertices[1] - edgeVertices[0]).normalized();
				Vector3 majorAxis = (triangleEdge * (triangleEdge.dot(cAxis)) + planeN[0] * (planeN[0].dot(cAxis))).normalized();
				Vector3 deepestPoint; // The deepest point of penetration that has been found.
				double majorRadius = farthestIntersectionLineCylinder(center, majorAxis, cr, cInverseTransform,
					&deepestPoint);
				SURGSIM_ASSERT(isValid(majorRadius)) << __func__ << " " << __LINE__ <<
					"The major radius of the ellipse should be a valid number." << std::setprecision(16) <<
					"\nv[0]: " << v[0].transpose() << "\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() <<
					"\ncr: " << cr << "\ntop: " << top.transpose() << "\nbottom: " << bottom.transpose();

				if (std::abs(majorAxis.dot(triangleEdge)) > Geometry::DistanceEpsilon)
				{ // majorRadius * majorAxis is not the deepest point because the ellipse is angled.
					auto minorAxis = planeN[i + 1].cross(majorAxis);
					T minorRadius =
						farthestIntersectionLineCylinder(center, minorAxis, cr, cInverseTransform);
					SURGSIM_ASSERT(isValid(minorRadius)) << __func__ << " " << __LINE__ <<
						"The minor radius of the ellipse should be a valid number." << std::setprecision(16) <<
						"\nv[0]: " << v[0].transpose() << "\nv[1]: " << v[1].transpose() <<
						"\nv[2]: " << v[2].transpose() << "\ncr: " << cr << "\ntop: " << top.transpose() <<
						"\nbottom: " << bottom.transpose();
					deepestPoint = pointWithTangentOnEllipse(center, majorAxis, minorAxis, majorRadius, minorRadius,
						triangleEdge);
				}

				auto edgeLength = (edgeVertices[1] - edgeVertices[0]).norm();
				double deepestPointDotEdge = triangleEdge.dot(deepestPoint - edgeVertices[0]);
				if (deepestPointDotEdge <= -Geometry::DistanceEpsilon ||
					deepestPointDotEdge >= edgeLength + Geometry::DistanceEpsilon)
				{
					// In this case, the intersection of the cylinder with the triangle edge plane gives an ellipse
					// that is close to a triangle corner and the deepest penetration point on the ellipse is
					// actually outside the triangle.
					Vector3 edgeVertex = (deepestPointDotEdge < 0.0) ? edgeVertices[0] : edgeVertices[1];
					Vector3 bottomEdgeVertex = edgeVertex + planeN[0] * (bottom.dot(planeN[0]) + planeD[0]);
					Vector3 pt0;
					Vector3 pt1;
					if (distanceSegmentSegment(top, bottom, edgeVertex, bottomEdgeVertex, &pt0, &pt1) <= cr)
					{
						farthestIntersectionLineCapsule(top, cAxis, cLength, cr, edgeVertex, -planeN[0],
							cTransform, cInverseTransform, penetrationPointCapsule, penetrationPointCapsuleAxis);
						return;
					}
				}

				if (isPointInsideTriangle(deepestPoint, v[0], v[1], v[2], planeN[0]))
				{
					*penetrationPointCapsule = deepestPoint;
					distancePointSegment(*penetrationPointCapsule, top, bottom, penetrationPointCapsuleAxis);
					return;
				}
			}
		}

		Vector3 closestPoint;
		T dist = distancePointTriangle(top, v[0], v[1], v[2], &closestPoint);
		SURGSIM_ASSERT(dist < cr + Geometry::DistanceEpsilon) << __func__ << " " << __LINE__ <<
			"There is a Triangle-Capsule intersection, the axis touches the triangle, the bottom is further " <<
			"than radius from the the swept prism, and the deepest point is not on the cylinder.  So the " <<
			"penetration point must be on the top endpoint, but it is too far away." << std::setprecision(16) <<
			"\nv[0]: " << v[0].transpose() << "\nv[1]: " << v[1].transpose() << "\nv[2]: " << v[2].transpose() <<
			"\ntop: " << top.transpose() << "\nbottom: " << bottom.transpose() << "\ncr: " << cr;

		farthestIntersectionLineSphere(closestPoint, closestPoint - planeN[0], top, cr, penetrationPointCapsule);
		distancePointSegment(*penetrationPointCapsule, top, bottom, penetrationPointCapsuleAxis);
	}

	/// This function calculates the intersection of a line with a sphere, returning the point furthest along the vector
	/// from v1 to v2.
	/// \param v1 A point on the line.
	/// \param v2 A second point on the line.
	/// \param center The center of the sphere.
	/// \param radius The radius of the sphere.
	/// \param [out] point The point of intersection. If there are two intersections, this is the one most positive along (v2 - v1).
	/// \exception Asserts if there is no intersection.
	static void farthestIntersectionLineSphere(const Vector3& v1, const Vector3& v2, const Vector3& center, T radius,
		Vector3* point)
	{
		// sphere: (x - xc)^2 + (y - yc)^2  + (z - zc)^2 = r^2
		// line: x = x1 + (x2 - x1)t
		//		 y = y1 + (y2 - y1)t
		//		 z = z1 + (z2 - z1)t
		// Substitute line equations into sphere equation, to get a quadratic in t:
		// at^2 + bt + c = 0, where
		// a = (x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2
		// b = - 2[(x2 - x1)(xc - x1) + (y2 - y1)(yc - y1) + (z2 - z1)(z2 - z1)]
		// c = (xc - x1)^2 + (yc - y1)^2 + (zc - z1)^2 - r^2
		Vector3 delta = v2 - v1;
		Vector3 vc1 = center - v1;
		T a = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
		T b = -static_cast<T>(2) * (delta[0] * vc1[0] + delta[1] * vc1[1] + delta[2] * vc1[2]);
		T c = vc1[0] * vc1[0] + vc1[1] * vc1[1] + vc1[2] * vc1[2] - radius * radius;
		T discriminant = b * b - static_cast<T>(4) * a * c;

		if (discriminant < static_cast<T>(0))
		{
			// Cannot use a sqrt on a negative number. Push it to zero if it is small.
			SURGSIM_ASSERT(discriminant >= -Geometry::ScalarEpsilon) << __func__ << " " << __LINE__ <<
				"Failed to solve for intersection of line with a sphere." <<
				std::setprecision(16) << "\nv1: " << v1.transpose() << "\nv2: " << v2.transpose() <<
				"\ncenter: " << center.transpose() << "\nradius: " << radius;
			discriminant = static_cast<T>(0);
		}

		// We have two solutions. We want the larger value.
		T d = (-b / (static_cast<T>(2) * a)) + std::abs(std::sqrt(discriminant) / (static_cast<T>(2) * a));
		SURGSIM_ASSERT(isValid(d)) << __func__ << " " << __LINE__ <<
			"Failed to solve for intersection of line with a sphere." <<
			std::setprecision(16) << "\nv1: " << v1.transpose() << "\nv2: " << v2.transpose() <<
			"\ncenter: " << center.transpose() << "\nradius: " << radius;
		*point = v1 + delta * d;
	}

	///@{
	/// Triangle vertices and normal.
	Vector3 m_tv0;
	Vector3 m_tv1;
	Vector3 m_tv2;
	Vector3 m_tn;
	///@}
	///@{
	/// Capsule ends, axis , radius and length.
	Vector3 m_cvTop;
	Vector3 m_cvBottom;
	Vector3 m_cAxis;
	double m_cr;
	double m_cLength;
	///@}
	/// Distance between triangle and capsule
	double m_distance;
	///@{
	/// Contact info
	T m_penetrationDepth;
	Vector3 m_penetrationPointTriangle;
	Vector3 m_penetrationPointCapsule;
	Vector3 m_contactNormal;
	Vector3 m_penetrationPointCapsuleAxis;
	///@}
	/// The transform of the capsule
	RigidTransform3 m_cTransform;
	/// The inverse transform of the capsule
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
		calc(tv0, tv1, tv2, tn, cv0, cv1, cr);
	try {


	if (calc.isIntersecting())
	{
		calc.calculateContact(penetrationDepth, penetrationPointTriangle, penetrationPointCapsule,
			contactNormal, penetrationPointCapsuleAxis);
		return true;
	}
	}
	catch (std::exception e)
	{
		calc.toFile("faultycalc.bin");
		throw e;
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
