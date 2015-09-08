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
	else if (distance > EPSILON)
	{
		// The axis of the capsule does not intersect with the triangle, so a correction of (cr - distance) along the
		// closest points between the two, is the shortest contact correction possible.
		*contactNormal = *penetrationPointTriangle - *penetrationPointCapsule;
		contactNormal->normalize();
		*penetrationDepth = cr - distance;
		*penetrationPointCapsule += (*contactNormal * cr);
	}
	else
	{
		/* The axis of the capsule intersects with the triangle face.
		   Contact can be corrected either by moving along the capsule's normal or the triangle's normal. Calculate
		   both and choose the one with the lower penetration depth. This gives four possible intersection corrections:
		   - three along the capsule normal (explained below)
		   - one along the triangle normal
		   The lowest penetration depth out of these four is chosen as the contact data.
		*/
		Vector3 normal[4];
		Eigen::Matrix<T, 4, 1, MOpt> depth;
		Vector3 contactPointOnCapsule[4];
		Vector3 contactPointOnTriangle[4];

		/* Penetration depth along the capsule's normal:
		   Capsule can be moved in three directions to remove intersection, one for each edge of the triangle. For
		   each edge of the triangle, find the perpendicular vector for the edge and the capsule axis. This is the
		   capsule normal. The penetration of the capsule into the triangle along this direction for each edge is found
		   as follows:
		   - The closest point to the capsule axis on this edge is P.
		   - The projection of P on the capsule axis is PC.
		   - The penetrationPointCapsule (from above) is PPC.
		   - P, PC and PPC form a triangle.
		   - Clamp the point PC to be within the capsule axis segment, making the point CPC.
		   - From CPC, draw a vector along (PC -> P); This intersects the triangle at Q.
		   - (P, PC, PPC) and (Q, CPC, PPC) are similar triangles.
		   - Penetration depth is (distance(Q, CPC) + capsule radius), penetration normal is (CPC ->

		                                 _,x PC                      tn
		                            _,-'    \                        ^
		                       _,-'        _,x cv1, CPC              |
		           tv0   _,-'        _,-'    \                       |
		           P  x-'---------x-'----------x-PPC-------------------------------x tv2
		            tv1           Q             \
		                                         x cv0
		*/
		Vector3 v[3] = {tv0, tv1, tv2};

		// Capsule axis points from above the triangle to below the triangle.
		Vector3 capsuleAxis = (cv0 - cv1).normalized();
		auto capsuleAxisProjectedOnTn = capsuleAxis.dot(tn);
		if (capsuleAxisProjectedOnTn > 0.0)
		{
			capsuleAxis = -capsuleAxis;
			capsuleAxisProjectedOnTn = -capsuleAxisProjectedOnTn;
		}

		Vector3 pointOnCapsuleAxis;
		for (size_t i = 0; i < 3; ++i)
		{
			normal[i] = capsuleAxis.cross(v[i] - v[(i + 1) % 3]).normalized();
			SurgSim::Math::distanceSegmentSegment(cv0, cv1, v[i], v[(i + 1) % 3],
				&pointOnCapsuleAxis, &contactPointOnTriangle[i]);
			depth[i] = std::abs(normal[i].dot(pointOnCapsuleAxis - contactPointOnTriangle[i])) + cr;
			contactPointOnCapsule[i] = pointOnCapsuleAxis + normal[i] * cr;
		}

		/* Penetration depth along the triangle's normal:

		                            cv0
		                            /
		                           /
		                 tv1      / P
		          tv0 x---x------x------x tv2
		                        /
		                       / Q
		                     cv1

		   The axis of the capsule passes through the triangle. The line segment (P, cv1) is the part of the capsule
		   axis which is under the triangle. If the point cv1 is projected onto the plane of the triangle, it may or
		   may not be inside the triangle. If it is inside, then the deepest penetration point on the capsule axis is
		   cv1. If the projection of cv1 on the plane of the triangle is outside the triangle, then the line segment
		   (P, cv1) is clipped, such that the projection of the newly clipped edge (Q) is inside the triangle.
		   Note: P here is *penetrationPointCapsule and
		  		 Q is deeperEndOfCapsuleAxis, in the code below.
		*/
		normal[3] = -tn;
		Vector3 deeperEndOfCapsuleAxis = (cv0.dot(normal[3]) > cv1.dot(normal[3])) ? cv0 : cv1;
		if (std::abs(capsuleAxisProjectedOnTn) < (1.0 - EPSILON))
		{
			for (size_t i = 0; i < 3; ++i)
			{
				Vector3 planeNormal = tn.cross(v[(i + 1) % 3] - v[i]);
				planeNormal.normalize();
				double planeD = v[i].dot(planeNormal);
				double deeperEndOfCapsuleAxisD = deeperEndOfCapsuleAxis.dot(planeNormal) - planeD;
				if (deeperEndOfCapsuleAxisD < 0.0)
				{
					double penetrationPointCapsuleD = penetrationPointCapsule->dot(planeNormal) - planeD;
					double ratio = penetrationPointCapsuleD / (penetrationPointCapsuleD - deeperEndOfCapsuleAxisD);
					deeperEndOfCapsuleAxis = *penetrationPointCapsule +
						(deeperEndOfCapsuleAxis - *penetrationPointCapsule) * ratio;
					i = 3;
				}
			}

			/* We have found Q (see comment above). Now, the point on the capsule that is deepest inside the triangle
			   along the triangle normal is to be calculated. A ray starting from Q and going in the direction of the
			   -tn would intersect the capsule at this deepest point.

			                 tv1   P                       P   tv1
			        tv0 x-----x----x------x tv2      tv0 x-x---x-----------x tv2
			                      /                       /
			                     /                     Q x
			              ,     x Q              ,      /|
			              ,     |     ,          ,       |   ,
			               \,   |   ,/            \,     | ,/
			                 ' -x- '                ' -_-x'
			                    R                         R
			             case 1: unclipped                   case 2: clipped
			   
			   First, find the depth as if the capsule was a cylinder (see figure below). So, in the cross section, the
			   shape's profile can be considered as a rectangle. QR' makes an angle theta with PQ. If the shape was a
			   cylinder, then R' would be on the surface of the cylinder: then the distance of R' from PQ would be the
			   radius of the cylinder, and S is the closest point on the axis of the cylinder to the point R'. Now, in
			   triangle (S, Q, R'), |QR'| = |SR'| / sin(theta), and
			                           R' = Q + |QR'| * (normalized direction of ray from Q to R)
			   
			                 tv1   P
			        tv0 x-----x----x------x tv2
			                /     /     /
			               /     /     /
			              /     x Q   /
			             /     /|    /
			            /     / |   /
			           /     /  |  /
			          /     /   | /
			         /   S x,_  |/
			        /         '-x
			                     R'
			  
			   Note: Q is deeperEndOfCapsuleAxis,
			         R' is pointOnCapsule in the code below.
			*/
			depth[3] = cr / std::sin(std::acos(std::abs(capsuleAxisProjectedOnTn)));
			Vector3 pointOnCylinder = deeperEndOfCapsuleAxis + (normal[3] * depth[3]);

			// Now, clip QR' to be on the surface of the capsule. This is done by finding |QR|
			if (SurgSim::Math::distancePointSegment(pointOnCylinder, cv0, cv1, &pointOnCapsuleAxis) > cr)
			{
				depth[3] = cr;
			}

			// R = Q + |QR| * (normalized direction of ray from Q to R)
			contactPointOnCapsule[3] = deeperEndOfCapsuleAxis + normal[3] * depth[3];
			// Add the penetration depth of the vertex Q from the triangle plane.
			depth[3] += std::abs((deeperEndOfCapsuleAxis - *penetrationPointCapsule).dot(normal[3]));
			// Project R on the triangle plane to get the contact point on the triangle.
			contactPointOnTriangle[3] = contactPointOnCapsule[3] - normal[3] * depth[3];
		}
		else
		{
			// The capsule axis is perpendicular to the triangle plane.
			depth[3] = cr + std::abs((deeperEndOfCapsuleAxis - *penetrationPointCapsule).dot(normal[3]));
			contactPointOnCapsule[3] = deeperEndOfCapsuleAxis + normal[3] * cr;
			contactPointOnTriangle[3] = *penetrationPointTriangle;
		}

		// Find the lowest of the penetration depths:
		size_t minIndex;
		depth.minCoeff(&minIndex);

		*penetrationDepth = depth[minIndex];
		*penetrationPointTriangle = contactPointOnTriangle[minIndex];
		*penetrationPointCapsule = contactPointOnCapsule[minIndex];
		*contactNormal = normal[minIndex];
	}

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
	return calculateContactTriangleTriangle(tv0, tv1, tv2, tn, cv0, cv1, cr, penetrationDepth,
		penetrationPoint0, penetrationPoint1, contactNormal);
}

} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_TRIANGLECAPSULECONTACTCALCULATION_INL_H