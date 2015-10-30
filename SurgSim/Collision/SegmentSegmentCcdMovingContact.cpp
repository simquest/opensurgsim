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

#include "SurgSim/Collision/SegmentSegmentCcdMovingContact.h"

#include <vector>

#include "SurgSim/Collision/SegmentSegmentCcdIntervalCheck.h"
#include "SurgSim/Collision/SegmentSegmentCcdStaticContact.h"
#include "SurgSim/Framework/LogMacros.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

SegmentSegmentCcdMovingContact::SegmentSegmentCcdMovingContact() :
	m_distanceEpsilon(1.0e-09), m_logger(Framework::Logger::getLogger("CCDMovingContactLog"))
{
}

bool SegmentSegmentCcdMovingContact::collideMovingSegmentSegment(
	const std::array<Math::Vector3d, 2>& pt0Positions,
	const std::array<Math::Vector3d, 2>& pt1Positions,
	const std::array<Math::Vector3d, 2>& qt0Positions,
	const std::array<Math::Vector3d, 2>& qt1Positions,
	double thicknessEpsilon,
	double timePrecisionEpsilon,
	double* t, double* r, double* s, Math::Vector3d* pToQDir)
{
	return collideMovingSegmentSegment(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
									   thicknessEpsilon / 2.0, thicknessEpsilon / 2.0, timePrecisionEpsilon,
									   t, r, s, pToQDir);
}

bool SegmentSegmentCcdMovingContact::collideMovingSegmentSegment(
	const std::array<Math::Vector3d, 2>& pt0Positions,
	const std::array<Math::Vector3d, 2>& pt1Positions,
	const std::array<Math::Vector3d, 2>& qt0Positions,
	const std::array<Math::Vector3d, 2>& qt1Positions,
	double thicknessP,
	double thicknessQ,
	double timePrecisionEpsilon,
	double* t, double* r, double* s, Math::Vector3d* pToQDir)
{
	bool ans = collideSegmentSegmentBaseCase(
				   pt0Positions, pt1Positions, qt0Positions, qt1Positions,
				   thicknessP, thicknessQ, timePrecisionEpsilon, t, r, s);

	if (ans)
	{
		// Calculate the contact points at t.
		Math::Vector3d contactPointP = Math::interpolate(
										   Math::interpolate(pt0Positions[0], pt0Positions[1], *r),
										   Math::interpolate(pt1Positions[0], pt1Positions[1], *r),
										   *t);
		Math::Vector3d contactPointQ = Math::interpolate(
										   Math::interpolate(qt0Positions[0], qt0Positions[1], *s),
										   Math::interpolate(qt1Positions[0], qt1Positions[1], *s),
										   *t);
		*pToQDir = contactPointQ - contactPointP;
	}
	return ans;
}

bool SegmentSegmentCcdMovingContact::collideSegmentSegmentBaseCase(
	const std::array<Math::Vector3d, 2>& pT0,
	const std::array<Math::Vector3d, 2>& pT1,
	const std::array<Math::Vector3d, 2>& qT0,
	const std::array<Math::Vector3d, 2>& qT1,
	double thicknessP,
	double thicknessQ,
	double timePrecisionEpsilon,
	double* t, double* r, double* s)
{
	bool collisionFound = false;

	const double parallelEpsilon = 1e-9;
	const double degenerateEpsilon = 1e-10;
	const double coplanarEpsilon = 1e-18;

	const double parallelEpsilon2 = parallelEpsilon * parallelEpsilon;

	Math::Vector3d p0t0p1t0 = pT0[1] - pT0[0];
	Math::Vector3d q0t0q1t0 = qT0[1] - qT0[0];

	Math::Vector3d p0t1p1t1 = pT1[1] - pT1[0];
	Math::Vector3d q0t1q1t1 = qT1[1] - qT1[0];

	//#################################################################
	// 1st) Case of parallel segment through time step (at t=0 and t=1)
	//
	// Check if the cross product is near zero at both t0 and t1.
	Math::Vector3d p0t0p1t0_vec_q0t0q1t0 = p0t0p1t0.cross(q0t0q1t0);
	Math::Vector3d p0t1p1t1_vec_q0t1q1t1 = p0t1p1t1.cross(q0t1q1t1);
	double p0t0p1t0_vec_q0t0q1t0_SQ = p0t0p1t0_vec_q0t0q1t0.squaredNorm();
	double p0t1p1t1_vec_q0t1q1t1_SQ = p0t1p1t1_vec_q0t1q1t1.squaredNorm();
	if (p0t0p1t0_vec_q0t0q1t0_SQ < parallelEpsilon2 && p0t1p1t1_vec_q0t1q1t1_SQ < parallelEpsilon2)
	{
		SURGSIM_LOG_WARNING(m_logger) <<
									  "Segments are parallel at start and end of time step. " <<
									  "Handling with parallel method rather than general method.";

		// Either it collides at t=0 or we do a dichotomy to find intersection in [0..1]
		if (m_staticTest.collideStaticSegmentSegment(pT0, qT0, thicknessP, thicknessQ, r, s))
		{
			t = 0;
			collisionFound = true;
		}
		else
		{
			collisionFound = collideSegmentSegmentParallelCase(
								 pT0, // Segment 1 at t=0
								 pT1, // Segment 1 at t=1
								 qT0, // Segment 2 at t=0
								 qT1, // Segment 2 at t=1
								 0.0, 1.0,
								 thicknessP, thicknessQ,
								 timePrecisionEpsilon,
								 t, r, s);
		}
	}
	else
	{
		//#################################################################
		// 2nd) Case of coplanar segment through time step (at t=0  and t=1)

		// Calculate the normal of p X q for at least one of the end points of
		// q. Do this at both t=0 and t=1.
		Math::Vector3d pt0Xqt0 = Math::robustCrossProduct(pT0, qT0, degenerateEpsilon);
		Math::Vector3d pt1Xqt1 = Math::robustCrossProduct(pT1, qT1, degenerateEpsilon);

		// TODO(wturner): Currently set to (10^-9), but has been as high as (10^-7) in previous versions.
		// verify that this value works as intended. If not, we will need to add and set another
		// member variable for the additional threshold.
		normalizeSegmentsConsistently(&q0t0q1t0, &q0t1q1t1, m_distanceEpsilon);

		// Do the segments remain coplanar all the time ?
		if (std::fabs(pt0Xqt0.dot(q0t0q1t0)) < coplanarEpsilon &&
			std::fabs(pt1Xqt1.dot(q0t1q1t1)) < coplanarEpsilon)
		{
			SURGSIM_LOG_WARNING(m_logger) << "Segments are coplanar at start and end of " <<
										  "the time step. Handling with coplanar method.";

			if (m_staticTest.collideStaticSegmentSegment(pT0, qT0, thicknessP, thicknessQ, r, s))
			{
				t = 0;
				collisionFound = true;
			}
			else
			{
				// At this point, {r,s} are computed for t=0 !
				collisionFound = collideSegmentSegmentCoplanarCase(
									 pT0, // Segment 1 at t=0
									 pT1, // Segment 1 at t=1
									 qT0, // Segment 2 at t=0
									 qT1, // Segment 2 at t=1
									 0.0, 1.0, // Interval boundaries
									 timePrecisionEpsilon,
									 thicknessP, thicknessQ,
									 t, r, s);
			}
		}
		else
		{
			//#################################################################
			// 3rd) General case
			SegmentSegmentCcdIntervalCheck state(pT0, pT1, qT0, qT1,
												 thicknessP, thicknessQ,
												 timePrecisionEpsilon, -1);

			collisionFound = collideSegmentSegmentGeneralCase(
								 state,
								 0.0, 1.0, // Look inside the interval t [0..1]
								 t, r, s);
		}
	}
	return collisionFound;
}

bool SegmentSegmentCcdMovingContact::collideSegmentSegmentParallelCase(
	const std::array<Math::Vector3d, 2>& pT0,
	const std::array<Math::Vector3d, 2>& pT1,
	const std::array<Math::Vector3d, 2>& qT0,
	const std::array<Math::Vector3d, 2>& qT1,
	double a, double b,
	double thicknessP,
	double thicknessQ,
	double timePrecisionEpsilon,
	double* t, double* r, double* s, int depth /*= 0*/)
{
	// Geometry at time t=b
	Math::Vector3d p0Tb = Math::interpolate(pT0[0], pT1[0], b); // p[0] interpolated at time b
	Math::Vector3d p1Tb = Math::interpolate(pT0[1], pT1[1], b); // p[1] interpolated at time b
	Math::Vector3d q0Tb = Math::interpolate(qT0[0], qT1[0], b); // q[0] interpolated at time b
	Math::Vector3d q1Tb = Math::interpolate(qT0[1], qT1[1], b); // q[1] interpolated at time b

	std::array<Vector3d, 2> pb = {p0Tb, p1Tb};
	std::array<Vector3d, 2> qb = {q0Tb, q1Tb};

	if (b - a < timePrecisionEpsilon)
	{
		// We know that no collision happened at t=a, that is why we recursed to this level, but
		// we believe that there is a collision in the interval. If our time precision is good enough,
		// the segments should be colliding at t=b. Otherwise, we are either moving too fast to detect
		// this collision, or something went wrong. Report false.
		if (m_staticTest.collideStaticSegmentSegment(pb, qb, thicknessP, thicknessQ, r, s))
		{
			*t = b;
			return true;
		}

		return false;
	}

	// Geometry at time t=a
	Math::Vector3d p0Ta = Math::interpolate(pT0[0], pT1[0], a); // p[0] interpolated at time a
	Math::Vector3d q0Ta = Math::interpolate(qT0[0], qT1[0], a); // q[0] interpolated at time a
	Math::Vector3d q1Ta = Math::interpolate(qT0[1], qT1[1], a); // q[1] interpolated at time a

	// Compute intermediate geometry..and work on subdivisions based on 1/nbSubPoint (instead of 1/2)
	//
	// During this next phase, we potentially take two passes through the system. In the first pass, we
	// do a quick check to see if we can find a collision at the end of any subdivision using our static
	// segment code. If we do, we can immediately recurse only on that subdivision to further refine
	// our time estimate.
	//
	// Otherwise, failing to find an easy detection, there is the possibility that we are moving
	// towards an intersection at the start of an interval and away at the end. Any possible
	// contact would be within the bracketed time frame, so if we detect it, we also recurse on the
	// candidate interval.
	//
	double t_i[SUB_POINTS_PARALLEL_CASE + 1];
	Math::Vector3d p0p0Proj[SUB_POINTS_PARALLEL_CASE + 1];

	Math::Vector3d p0Proj = Math::nearestPointOnLine(p0Ta, q0Ta, q1Ta);
	p0p0Proj[0] = p0Proj - p0Ta;
	t_i[0] = a;

	double deltaT = 1.0 / SUB_POINTS_PARALLEL_CASE;
	for (int i = 1 ; i <= SUB_POINTS_PARALLEL_CASE ; i++)
	{
		t_i[i] = a + (b - a) * (i * deltaT);
		double r_i;
		double s_i;

		// p and q at time i
		std::array<Math::Vector3d, 2> p_i = {Math::interpolate(pT0[0], pT1[0], t_i[i]),
											 Math::interpolate(pT0[1], pT1[1], t_i[i])
											};
		std::array<Math::Vector3d, 2> q_i = {Math::interpolate(qT0[0], qT1[0], t_i[i]),
											 Math::interpolate(qT0[1], qT1[1], t_i[i])
											};

		if (m_staticTest.collideStaticSegmentSegment(p_i, q_i, thicknessP, thicknessQ, &r_i, &s_i))
		{
			// Collision happens between [t[i-1]..t[i]]
			return collideSegmentSegmentParallelCase(
					   pT0, pT1, qT0, qT1, t_i[i - 1], t_i[i],
					   thicknessP, thicknessQ, timePrecisionEpsilon,
					   t, r, s,
					   depth + 1);
		}

		p0Proj = Math::nearestPointOnLine(p_i[0], q_i[0], q_i[1]);
		p0p0Proj[i] = p0Proj - p_i[0];
	}

	// No collision found at the discretization
	// Let analyze the variation of p1q1...if it switches direction, we more
	// likely passed a parallel intersection case in between !
	for (int i = 0; i < SUB_POINTS_PARALLEL_CASE; i++)
	{
		if (p0p0Proj[i].dot(p0p0Proj[i + 1]) < 0.0)
		{
			bool found = collideSegmentSegmentParallelCase(
							 pT0, pT1, qT0, qT1, t_i[i], t_i[i + 1],
							 thicknessP, thicknessQ, timePrecisionEpsilon,
							 t, r, s,
							 depth + 1);
			if (found)
			{
				return true;
			}
		}
	}
	return false;
}

bool SegmentSegmentCcdMovingContact::collideSegmentSegmentCoplanarCase(
	const std::array<Math::Vector3d, 2>& pT0, /* Segment 1 at t=0 */
	const std::array<Math::Vector3d, 2>& pT1, /* Segment 1 at t=1 */
	const std::array<Math::Vector3d, 2>& qT0, /* Segment 2 at t=0 */
	const std::array<Math::Vector3d, 2>& qT1, /* Segment 2 at t=1 */
	double a, double b, /* Interval boundaries */
	double timePrecisionEpsilon,
	double thickness_p, double thickness_q,
	double* t, double* r, double* s,
	int depth /*= 0*/)
{
	// Geometry at time t=b
	Math::Vector3d p0Tb = Math::interpolate(pT0[0], pT1[0], b); // p[0] interpolated at time b
	Math::Vector3d p1Tb = Math::interpolate(pT0[1], pT1[1], b); // p[1] interpolated at time b
	Math::Vector3d q0Tb = Math::interpolate(qT0[0], qT1[0], b); // q[0] interpolated at time b
	Math::Vector3d q1Tb = Math::interpolate(qT0[1], qT1[1], b); // q[1] interpolated at time b


	if (b - a < timePrecisionEpsilon)
	{
		bool collisionFound = false;

		std::array<Math::Vector3d, 2> pTb = {p0Tb, p1Tb};
		std::array<Math::Vector3d, 2> qTb = {q0Tb, q1Tb};

		// We do know that no collision happen at t=a.
		// We do know that a collision happen in between, but not sure exactly when...
		// and more likely the 2 segments are colliding at t=b
		if (m_staticTest.collideStaticSegmentSegment(pTb, qTb, thickness_p, thickness_q, r, s))
		{
			*t = b;
			collisionFound = true;
		}

		return collisionFound;
	}
	// Geometry at time t=a
	Math::Vector3d p0Ta = Math::interpolate(pT0[0], pT1[0], a); // p[0] interpolated at time a
	Math::Vector3d p1Ta = Math::interpolate(pT0[1], pT1[1], a); // p[1] interpolated at time a
	Math::Vector3d q0Ta = Math::interpolate(qT0[0], qT1[0], a); // q[0] interpolated at time a
	Math::Vector3d q1Ta = Math::interpolate(qT0[1], qT1[1], a); // q[1] interpolated at time a

	// Compute intermediate geometry..and work on dichotomy based on 1/nbSubPoint (instead of 1/2)
	double t_i[SUB_POINTS_COPLANAR_CASE + 1];
	Math::Vector3d normal[SUB_POINTS_COPLANAR_CASE + 1];
	double deltaT = 1.0 / SUB_POINTS_COPLANAR_CASE;
	double r_i[SUB_POINTS_COPLANAR_CASE + 1], s_i[SUB_POINTS_COPLANAR_CASE + 1];
	t_i[0] = a;
	r_i[0] = *r;
	s_i[0] = *s;
	normal[0] = (p1Ta - p0Ta).cross(q1Ta - q0Ta).normalized();

	for (int i = 1 ; i <= SUB_POINTS_COPLANAR_CASE ; i++)
	{
		t_i[i] = a + (b - a) * (i * deltaT);

		// p and q at time i
		std::array<Math::Vector3d, 2> p_i = {Math::interpolate(pT0[0], pT1[0], t_i[i]),
											 Math::interpolate(pT0[1], pT1[1], t_i[i])
											};
		std::array<Math::Vector3d, 2> q_i = {Math::interpolate(qT0[0], qT1[0], t_i[i]),
											 Math::interpolate(qT0[1], qT1[1], t_i[i])
											};

		if (m_staticTest.collideStaticSegmentSegment(p_i, q_i, thickness_p, thickness_q, &r_i[i], &s_i[i]))
		{
			*r = r_i[i - 1];
			*s = s_i[i - 1];
			return collideSegmentSegmentCoplanarCase(
					   pT0, // Segment 1 at t=0
					   pT1, // Segment 1 at t=1
					   qT0, // Segment 2 at t=0
					   qT1, // Segment 2 at t=1
					   t_i[i - 1], t_i[i], // Interval boundaries
					   timePrecisionEpsilon,
					   thickness_p, thickness_q,
					   t, r, s,
					   depth + 1);
		}
		normal[i] = (p_i[1] - p_i[0]).cross(q_i[1] - q_i[0]).normalized();
	}

	// Check for a flip or a change in normal. If one is detected, then a collision might occur within the
	// current interval. Recurse and check.
	for (int i = 1; i <= SUB_POINTS_COPLANAR_CASE; i++)
	{
		if (checkForCoplanarContactWithinInterval(r_i[i - 1], r_i[i], s_i[i - 1], s_i[i],
				normal[i - 1], normal[i]))
		{
			double old_r = *r;
			double old_s = *s;
			*r = r_i[i - 1];
			*s = s_i[i - 1];
			bool found = collideSegmentSegmentCoplanarCase(
							 pT0, // Segment 1 at t=0
							 pT1, // Segment 1 at t=1
							 qT0, // Segment 2 at t=0
							 qT1, // Segment 2 at t=1
							 t_i[i - 1], t_i[i], // Interval boundaries
							 timePrecisionEpsilon,
							 thickness_p, thickness_q,
							 t, r, s,
							 depth + 1);
			if (found)
			{
				return true;
			}
			*r = old_r;
			*s = old_s;
		}
	}

	return false;
}

bool SegmentSegmentCcdMovingContact::collideSegmentSegmentGeneralCase(
	const SegmentSegmentCcdIntervalCheck& state,
	double a, double b, // Interval boundaries
	double* t, double* r, double* s,
	int depth)
{
	Math::Interval<double> range(a, b);

	if (state.possibleCollisionTestWithThickness(range) !=
		SegmentSegmentCcdIntervalCheck::IntervalCheckPossibleCollision)
	{
		return false;
	}

	// Recursion bottoms out at time precision.
	if (b - a < state.timePrecisionEpsilon())
	{
		bool collisionFound = false;

		std::array<Math::Vector3d, 2> pTb = {state.motionP1().atTime(b), state.motionP2().atTime(b)};
		std::array<Math::Vector3d, 2> qTb = {state.motionQ1().atTime(b), state.motionQ2().atTime(b)};

		// The recursion has bottomed out, and we should have already detected if we are colliding at t=0.
		// Make one final check at the other end of the interval and end the recursion.
		if (m_staticTest.collideStaticSegmentSegment(pTb, qTb, state.thicknessP(),
				state.thicknessQ(), r, s))
		{
			*t = b;
			collisionFound = true;
		}

		return collisionFound;
	}

	// Otherwise, recursion has not yet bottomed out, go down one more level.
	double midpoint = (a + b) * 0.5;

	// Test 1st semi-interval [a, (a + b) / 2]
	if (collideSegmentSegmentGeneralCase(
			state,
			a, midpoint,
			t, r, s,
			depth + 1))
	{
		return true;
	}

	// Test 2nd semi-interval [(a + b) / 2, b]
	if (collideSegmentSegmentGeneralCase(
			state,
			midpoint, b,
			t, r, s,
			depth + 1))
	{
		return true;
	}

	return false;
}

void SegmentSegmentCcdMovingContact::normalizeSegmentsConsistently(Math::Vector3d* t0, Math::Vector3d* t1,
		double epsilon) const
{
	// safely normalize t0 and t1. We will need to calculate
	// dot products against them to test orthogonality with p X q at
	// the two time points.
	double norm_t0 = t0->norm();
	double norm_t1 = t1->norm();

	if (norm_t0 >= epsilon)
	{
		*t0 *= 1.0 / norm_t0;
		if (norm_t1 >= epsilon)
		{
			// t0 and t1 both good
			*t1 *= 1.0 / norm_t1;
		}
		else
		{
			// t0 good, t1 bad
			*t1 = *t0;	// t1 <- t0
			SURGSIM_LOG_WARNING(m_logger) <<
										  "Segment is degenerate at time 1. Using time 0 " <<
										  "value for both coplanarity tests.";
		}
	}
	else if (norm_t1 >= epsilon)
	{
		// t1 good, t0 bad
		*t1 *= 1.0 / norm_t1;
		*t0 = *t1;	// t0 <- t1
		SURGSIM_LOG_WARNING(m_logger) <<
									  "Segment is degenerate at time 0. Using time 1 value for " <<
									  "both coplanarity tests.";
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) <<
									  "Segment is degenerate at time 0 and time 1. Unable to " <<
									  "normalize for coplanarity tests.";
	}
}

bool SegmentSegmentCcdMovingContact::checkForCoplanarContactWithinInterval(
	double rCurrent, double rNext,
	double sCurrent, double sNext,
	const Math::Vector3d& nCurrent, const Math::Vector3d& nNext) const
{
// TODO(wturner): Given the original code, this test IS ALWAYS TRUE as r and s are clamped to [0, 1]
// and numberSubpoints > 1. I do not think this was operational and that may explain the extraordinarily
// small epsilon (1.0e-18) being used a the gatekeeper to this function:
//
//	return (((rCurrent >= 0 && rCurrent < 1) || (rCurrent < 0 && rNext >= 0) || (rCurrent > 1 && rNext <= 1)) &&
//			((sCurrent >= 0 && sCurrent < 1) || (sCurrent < 0 && sNext >= 0) || (sCurrent > 1 && sNext <= 1)) &&
//			rCurrent * rNext > -(double)numberSubpoints * 0.5
//			&& sCurrent * sNext > -(double)numberSubpoints * 0.5) ||
//		   (nCurrent.dot(nNext) < 0.0);

	return (((rCurrent > 0.0 && rCurrent < 1.0) || (rCurrent == 0.0 && rNext > 0) ||
			 (std::abs(rCurrent - 1.0) < m_distanceEpsilon && rNext < 1.0)) &&
			((sCurrent > 0.0 && sCurrent < 1.0) || (sCurrent == 0.0 && sNext > 0) ||
			 (std::abs(sCurrent - 1.0) < m_distanceEpsilon && sNext < 1.0))) ||
		   (nCurrent.dot(nNext) < 0.0);
}

}; // namespace Collision
}; // namespace SurgSim
