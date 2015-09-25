// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/SegmentSegmentCcdIntervalCheck.h"

namespace SurgSim
{
namespace Collision
{
// The order of initialization MUST match the necessary initialization order --
// if member B is calculated from member A, then B must of course be initialized after A in
// the constructor, and B must be declared after A in here the class definition.
SegmentSegmentCcdIntervalCheck::SegmentSegmentCcdIntervalCheck(
	const std::array<Math::Vector3d, 2>& pT0,
	const std::array<Math::Vector3d, 2>& pT1,
	const std::array<Math::Vector3d, 2>& qT0,
	const std::array<Math::Vector3d, 2>& qT1,
	double thicknessP, double thicknessQ,
	double timePrecisionEpsilon,  double distanceEpsilon)
	: m_motionP1(pT0[0], pT1[0]), m_motionP2(pT0[1], pT1[1]), m_motionQ1(qT0[0], qT1[0]), m_motionQ2(qT0[1], qT1[1]),
	  m_relativeP1Q1(m_motionQ1 - m_motionP1),
	  m_relativeQ1Q2(m_motionQ2 - m_motionQ1),
	  m_relativeP1P2(m_motionP2 - m_motionP1),
	  m_P1Q1_P1P2_Q1Q2(Math::analyticTripleProduct(m_relativeP1Q1, m_relativeP1P2, m_relativeQ1Q2)),
	  m_P1P2_P1Q1(Math::analyticDotProduct(m_relativeP1P2, m_relativeP1Q1)),
	  m_Q1Q2_P1Q1(Math::analyticDotProduct(m_relativeQ1Q2, m_relativeP1Q1)),
	  m_P1P2_Q1Q2(Math::analyticDotProduct(m_relativeP1P2, m_relativeQ1Q2)),
	  m_P1P2_sq(Math::analyticMagnitudeSquared(m_relativeP1P2)),
	  m_Q1Q2_sq(Math::analyticMagnitudeSquared(m_relativeQ1Q2)),
	  m_P1P2xQ1Q2_x(Math::analyticCrossProductXAxis(m_relativeP1P2, m_relativeQ1Q2)),
	  m_P1P2xQ1Q2_y(Math::analyticCrossProductYAxis(m_relativeP1P2, m_relativeQ1Q2)),
	  m_P1P2xQ1Q2_z(Math::analyticCrossProductZAxis(m_relativeP1P2, m_relativeQ1Q2)),
	  m_thicknessP(thicknessP), m_thicknessQ(thicknessQ),
	  m_timePrecisionEpsilon(timePrecisionEpsilon), m_distanceEpsilon(distanceEpsilon),
	  m_volumeEpsilonTimes6(0), m_muNuEpsilon(0)
{
}

// Accessors
const Math::LinearMotionND<double, 3>& SegmentSegmentCcdIntervalCheck::motionP1() const
{
	return m_motionP1;
}

const Math::LinearMotionND<double, 3>& SegmentSegmentCcdIntervalCheck::motionP2() const
{
	return m_motionP2;
}

const Math::LinearMotionND<double, 3>& SegmentSegmentCcdIntervalCheck::motionQ1() const
{
	return m_motionQ1;
}

const Math::LinearMotionND<double, 3>& SegmentSegmentCcdIntervalCheck::motionQ2() const
{
	return m_motionQ2;
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::p1T0() const
{
	return m_motionP1.getStart();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::p1T1() const
{
	return m_motionP1.getEnd();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::p2T0() const
{
	return m_motionP2.getStart();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::p2T1() const
{
	return m_motionP2.getEnd();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::q1T0() const
{
	return m_motionQ1.getStart();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::q1T1() const
{
	return m_motionQ1.getEnd();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::q2T0() const
{
	return m_motionQ2.getStart();
}

Math::Vector3d SegmentSegmentCcdIntervalCheck::q2T1() const
{
	return m_motionQ2.getEnd();
}

const Math::PolynomialValues<double, 3>& SegmentSegmentCcdIntervalCheck::P1Q1_P1P2_Q1Q2() const
{
	return m_P1Q1_P1P2_Q1Q2;
}

const Math::PolynomialValues<double, 2>& SegmentSegmentCcdIntervalCheck::P1P2_P1Q1() const
{
	return m_P1P2_P1Q1;
}

const Math::PolynomialValues<double, 2>& SegmentSegmentCcdIntervalCheck::Q1Q2_P1Q1() const
{
	return m_Q1Q2_P1Q1;
}

const Math::PolynomialValues<double, 2>& SegmentSegmentCcdIntervalCheck::P1P2_Q1Q2() const
{
	return m_P1P2_Q1Q2;
}

const Math::PolynomialValues<double, 2>& SegmentSegmentCcdIntervalCheck::P1P2_sq() const
{
	return m_P1P2_sq;
}

const Math::PolynomialValues<double, 2>& SegmentSegmentCcdIntervalCheck::Q1Q2_sq() const
{
	return m_Q1Q2_sq;
}

Math::Interval<double> SegmentSegmentCcdIntervalCheck::crossValueOnInterval(
	const Math::Interval<double>& range) const
{
	return (
			   m_P1P2xQ1Q2_x.valuesOverInterval(range).square() +
			   m_P1P2xQ1Q2_y.valuesOverInterval(range).square() +
			   m_P1P2xQ1Q2_z.valuesOverInterval(range).square()
		   );
}

double SegmentSegmentCcdIntervalCheck::thicknessP() const
{
	return m_thicknessP;
}
double SegmentSegmentCcdIntervalCheck::thicknessQ() const
{
	return m_thicknessQ;
}

void SegmentSegmentCcdIntervalCheck::setTimePrecisionEpsilon(double epsilon)
{
	m_timePrecisionEpsilon = epsilon;
}

void SegmentSegmentCcdIntervalCheck::setDistanceEpsilon(double epsilon)
{
	m_distanceEpsilon = epsilon;
}

void SegmentSegmentCcdIntervalCheck::setTripleProductEpsilon(double epsilon)
{
	m_volumeEpsilonTimes6 = epsilon;
}

void SegmentSegmentCcdIntervalCheck::setMuNuEpsilon(double epsilon)
{
	m_muNuEpsilon = epsilon;
}

double SegmentSegmentCcdIntervalCheck::timePrecisionEpsilon() const
{
	return m_timePrecisionEpsilon;
}

double SegmentSegmentCcdIntervalCheck::distanceEpsilon() const
{
	return m_distanceEpsilon;
}

double SegmentSegmentCcdIntervalCheck::tripleProductEpsilon() const
{
	return m_volumeEpsilonTimes6;
}

double SegmentSegmentCcdIntervalCheck::muNuEpsilon() const
{
	return m_muNuEpsilon;
}

SegmentSegmentCcdIntervalCheck::IntervalCheckResults SegmentSegmentCcdIntervalCheck::possibleCollisionTestNoThickness(
	const Math::Interval<double>& range) const
{
	Math::Interval<double> P1Q1_P1P2_Q1Q2_values = P1Q1_P1P2_Q1Q2().valuesOverInterval(range);
	Math::Interval<double> P1P2_P1Q1_values = P1P2_P1Q1().valuesOverInterval(range);
	Math::Interval<double> Q1Q2_P1Q1_values = Q1Q2_P1Q1().valuesOverInterval(range);
	Math::Interval<double> P1P2_Q1Q2_values = P1P2_Q1Q2().valuesOverInterval(range);
	Math::Interval<double> P1P2_sq_values = P1P2_sq().valuesOverInterval(range);
	Math::Interval<double> Q1Q2_sq_values = Q1Q2_sq().valuesOverInterval(range);
	Math::Interval<double> crossProductSquared_values = crossValueOnInterval(range);

	P1Q1_P1P2_Q1Q2_values.addThickness(m_volumeEpsilonTimes6);

	bool lineDistanceIsZero = P1Q1_P1P2_Q1Q2_values.containsZero();
	if (!lineDistanceIsZero)
	{
		return IntervalCheckNoCollisionVolume;  // collision check #1 says no collision
	}

	// [The following eqns and their derivation are written up in
	// the document Segment-segmentCCDlocationcheck.pdf in the Assembla OSS Files page. See link:
	// https://www.assembla.com/spaces/OpenSurgSim/documents/dce2Euy6Cr5znOdmr6CpXy/download/dce2Euy6Cr5znOdmr6CpXy
	// 0 = \mu (P1P2 x Q1Q2)^2 - (P1P2 * P1Q1) Q1Q2^2 + (Q1Q2 * P1Q1) (P1P2 * Q1Q2)
	// 0 = \nu (P1P2 x Q1Q2)^2 - (Q1Q2 * P1Q1) P1P2^2 + (P1P2 * P1Q1) (P1P2 * Q1Q2)

	// Calculate the values of the \mu and \nu terms in the equations above; note that valid \mu and \nu are in [0,1].
	// We make use of the fact that I*0 = [0,0] and I*1 = I, so I*[0,1] can be found by simply extending I to include 0.
	Math::Interval<double> muNuTerm_values = crossProductSquared_values;
	muNuTerm_values.extendToInclude(0);
	// Add an epsilon to the result, to handle possible numerical noise.
	muNuTerm_values.addThickness(m_muNuEpsilon);

	Math::Interval<double> muExpression = muNuTerm_values - P1P2_P1Q1_values * Q1Q2_sq_values + Q1Q2_P1Q1_values *
										  P1P2_Q1Q2_values;
	Math::Interval<double> nuExpression = muNuTerm_values + Q1Q2_P1Q1_values * P1P2_sq_values - P1P2_P1Q1_values *
										  P1P2_Q1Q2_values;

	bool segmentLocationsMakeCollisionPossible = muExpression.containsZero() && nuExpression.containsZero();
	if (!segmentLocationsMakeCollisionPossible)
	{
		return IntervalCheckNoCollisionEndpoints;  // collision check #2 says no collision
	}

	return IntervalCheckPossibleCollision;  // as far as we know, a collision is possible
}

SegmentSegmentCcdIntervalCheck::IntervalCheckResults SegmentSegmentCcdIntervalCheck::possibleCollisionTestWithThickness(
	const Math::Interval<double>& range) const
{
	Math::Interval<double> P1Q1_P1P2_Q1Q2_values = P1Q1_P1P2_Q1Q2().valuesOverInterval(range);
	Math::Interval<double> P1P2_P1Q1_values      = P1P2_P1Q1().valuesOverInterval(range);
	Math::Interval<double> Q1Q2_P1Q1_values      = Q1Q2_P1Q1().valuesOverInterval(range);
	Math::Interval<double> P1P2_Q1Q2_values      = P1P2_Q1Q2().valuesOverInterval(range);
	Math::Interval<double> P1P2_sq_values        = P1P2_sq().valuesOverInterval(range);
	Math::Interval<double> Q1Q2_sq_values        = Q1Q2_sq().valuesOverInterval(range);
	Math::Interval<double> P1P2xQ1Q2_sq_values   = crossValueOnInterval(range);

	// Now we need to account for thickness.  [Look at Docs/Collision detection/Segment-segment
	// CCD with thickness.pdf for details!]
	double maxLengthP = std::sqrt(P1P2_sq_values.getMax());
	double maxLengthQ = std::sqrt(Q1Q2_sq_values.getMax());
	// Figure out some upper bounds on the value of | P1P2 x Q1Q2 |.
	// TO DO: figure out if both of the bounds are effective, i.e. if it makes sense to keep them both.
	// (My guess would be that the first is more effective for perpendicular segments, second for near-parallel.)
	double crossProductUpperBound1  = maxLengthP * maxLengthQ;             // ignores the sine of the angle
	double crossProductUpperBound2  = std::sqrt(P1P2xQ1Q2_sq_values.getMax());  // uses Interval_nD -> NOT necessarily tight
	double crossProductUpperBound = std::min(crossProductUpperBound1,
									crossProductUpperBound2);  // pick the LOWEST of the upper bounds...
	// Compute the upper bound on the volume that the tetrahedron P1P2Q1Q2 would have if the segments were
	// just coming into contact. (What we actually calculate is the bound on the triple product, which is
	// six times the tetrahedron volume.)
	double touchingVolumeUpperBound = crossProductUpperBound * (thicknessP() + thicknessQ());

	// Just to be clear here: The triple product P1Q1_P1P2_Q1Q2_values is the 6*volume of the
	// tetrahedron defined by [p1, p2, q1, q2] over the interval. The value touchingVolumeUpperBound
	// is an upper bound of 6*volume for the tetrahedron if the the two segments just reach contact.
	// If the interval contains 0, then somewhere in the interval, the two volumes meet and we have
	// contact at that point.
	P1Q1_P1P2_Q1Q2_values.addThickness(touchingVolumeUpperBound);
	bool lineDistanceIsSmallEnough = P1Q1_P1P2_Q1Q2_values.containsZero();
	if (!lineDistanceIsSmallEnough)
	{
		return IntervalCheckNoCollisionVolume;  // collision check #1 says no collision
	}

	// If we detected that a collision is possible based on segment orientation, then we will make one more
	// check to verify that the actual points of closest approach live on the segments.
	double minLengthP = std::sqrt(P1P2_sq_values.getMin());
	double minLengthQ = std::sqrt(Q1Q2_sq_values.getMin());
	Math::Interval<double> weightCoefficient = P1P2xQ1Q2_sq_values;

	// So, if we consider the endpoints, then the lambda and \mu overshoots extend the physical dimension
	// to include the end caps of the capsules. The division of the radii by minLengthX scales the implicit
	// overshoot proportionately to the length of the vector. E.g. so if the radii of the segments combine
	// to be 10% of the length of segment P, then the lambda interval will be extended by 10% = 0.1 at
	// the beginning and end, i.e. [-0.1, 1.1]
	double lambdaOvershoot = (thicknessP() + thicknessQ()) / minLengthP;
	Math::Interval<double> lambdaInterval = Math::Interval<double>(-lambdaOvershoot, 1. + lambdaOvershoot);
	Math::Interval<double> lambdaExpression = lambdaInterval * weightCoefficient - P1P2_P1Q1_values * Q1Q2_sq_values +
			Q1Q2_P1Q1_values * P1P2_Q1Q2_values;

	double muOvershoot = (thicknessP() + thicknessQ()) / minLengthQ;
	Math::Interval<double> muInterval = Math::Interval<double>(-muOvershoot, 1. + muOvershoot);
	Math::Interval<double> muExpression = muInterval * weightCoefficient + Q1Q2_P1Q1_values * P1P2_sq_values -
										  P1P2_P1Q1_values * P1P2_Q1Q2_values;

	bool segmentLocationsMakeCollisionPossible = lambdaExpression.containsZero() && muExpression.containsZero();
	if (!segmentLocationsMakeCollisionPossible)
	{
		return IntervalCheckNoCollisionEndpoints;  // collision check #2 says no collision
	}

	return IntervalCheckPossibleCollision;  // as far as we know, a collision is possible
}

}; // namespace Collision
}; // namespace SurgSim
