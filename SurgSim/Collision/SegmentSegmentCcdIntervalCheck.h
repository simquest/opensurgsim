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

#ifndef SURGSIM_COLLISION_SEGMENTSEGMENTINTERVALCCDCHECK_H
#define SURGSIM_COLLISION_SEGMENTSEGMENTINTERVALCCDCHECK_H

#include "SurgSim/Math/LinearMotionArithmetic.h"
#include "SurgSim/Math/PolynomialValues.h"

namespace SurgSim
{
namespace Collision
{
///
/// SegmentSegmentCcdIntervalCheck uses the Interval classes including the LinearMotion
/// and Polynomial families to quickly determine if there is a possible collision between
/// two moving segments over a specified time interval. The time interval
/// under consideration is defined as a subset of the parametric time interval [0, 1].
///
/// Details of the actual time of collision and the implementation of the recursion
/// strategy are at a higher level.
///
/// \sa Interval<T>, IntervalND<T, N>, LinearMotion<T, N>, and Polynomial<T, N>
///
class SegmentSegmentCcdIntervalCheck
{
public:

	/// Enum
	/// Possible interval check return values. IntervalCheckPossibleCollision indicates
	/// the given interval may have a collision between the segments, while
	/// IntervalCheckNoCollisionVolume indicates no collision based on a gross volume
	/// calculation and IntervalCheckNoCollisionEndpoints indicates that the endpoint
	/// check indicates that the segments do not overlap at closest approach.
	enum IntervalCheckResults
	{
		IntervalCheckPossibleCollision,
		IntervalCheckNoCollisionVolume,
		IntervalCheckNoCollisionEndpoints
	};

	/// Constructor
	/// \param pT0 Starting and ending vertices for segment p at time 0
	/// \param pT1 Starting and ending vertices for segment p at time 1
	/// \param qT0 Starting and ending vertices for segment q at time 0
	/// \param qT1 Starting and ending vertices for segment q at time 1
	/// \param thicknessP Radius of segment P
	/// \param thicknessQ Radius of segment Q
	/// \param timePrecisionEpsilon Desired time accuracy
	/// \param distanceEpsilon Desired distance accuracy
	SegmentSegmentCcdIntervalCheck(const std::array<Math::Vector3d, 2>& pT0,
								   const std::array<Math::Vector3d, 2>& pT1,
								   const std::array<Math::Vector3d, 2>& qT0,
								   const std::array<Math::Vector3d, 2>& qT1,
								   double thicknessP, double thicknessQ,
								   double timePrecisionEpsilon, double distanceEpsilon);
	/// @{
	/// Motion accessors
	/// \return the motion vector (value(t1) - value(t0)) for the segment endpoints p1, p2, q1, and q2, respectively.
	const Math::LinearMotionND<double, 3>& motionP1() const;
	const Math::LinearMotionND<double, 3>& motionP2() const;
	const Math::LinearMotionND<double, 3>& motionQ1() const;
	const Math::LinearMotionND<double, 3>& motionQ2() const;
	/// @}

	/// @{
	/// Endpoint accessors
	/// \return the motion vector [value(t0), value(t0)] for the segment endpoints p1, p2, q1, and q2, respectively.
	Math::Vector3d p1T0() const;
	Math::Vector3d p1T1() const;
	Math::Vector3d p2T0() const;
	Math::Vector3d p2T1() const;
	Math::Vector3d q1T0() const;
	Math::Vector3d q1T1() const;
	Math::Vector3d q2T0() const;
	Math::Vector3d q2T1() const;
	/// @}

	/// Triple product value
	/// \return the triple product of (Q1(t) - P1(t)) X (P2(t) - P1(t)) X (Q2(t) - Q1(t)) as a 3rd degree polynomial
	/// where P1, P2,Q1 and Q2 are time dependent positions for the segment endpoints.
	/// \note the triple product is equivalent to 6 x the volume of tetrahedron P1P2Q1Q2. The polynomial
	/// captures the variation in volume over the time interval.
	const Math::PolynomialValues<double, 3>& P1Q1_P1P2_Q1Q2() const;

	/// @{
	/// Dot product accessors for time dependent vertex positions P1(t), P2(t), Q1(t) and Q2(t)
	/// \return the dot product of the difference operators for the named endpoints
	const Math::PolynomialValues<double, 2>& P1P2_P1Q1() const;
	const Math::PolynomialValues<double, 2>& Q1Q2_P1Q1() const;
	const Math::PolynomialValues<double, 2>& P1P2_Q1Q2() const;
	/// @}

	/// @{
	/// Magnitude squared product accessors
	/// \return the squared magnitude of the difference operators for time dependent
	/// vertex positions P1(t), P2(t), Q1(t) and Q2(t).
	const Math::PolynomialValues<double, 2>& P1P2_sq() const;
	const Math::PolynomialValues<double, 2>& Q1Q2_sq() const;
	/// @}

	/// \param range the interval over which the cross product values are to be bounded.
	/// \return the minimum and maximum of (P2 - P1) X (Q2 - Q1) restricted to the interval range.
	Math::Interval<double> crossValueOnInterval(const Math::Interval<double>& range) const;

	/// @{
	/// Thickness accessors
	/// \return the thickness parameters for P and Q, respectively.
	double thicknessP() const;
	double thicknessQ() const;
	/// @}

	/// @{
	/// Algorithm epsilons. Set the epsilon values for the various member variables.
	/// \param epsilon the algorithm epsilon parameters for "close enough" decisions.
	void setTimePrecisionEpsilon(double epsilon);
	void setDistanceEpsilon(double epsilon);
	void setTripleProductEpsilon(double epsilon);
	void setMuNuEpsilon(double epsilon);
	/// @}

	/// @{
	/// Algorithm epsilons
	/// \return the algorithm epsilon parameters for "close enough" decisions.
	double timePrecisionEpsilon() const;
	double distanceEpsilon() const;
	double tripleProductEpsilon() const;
	double muNuEpsilon() const;
	/// @}

	/// Check if a collision is possible within a specified time interval assuming ideal (0 thickness) segments
	/// \param range the parametric [0, 1] time interval over which the collision is to be detected.
	/// \return IntervalCheckPossibleCollision, IntervalCheckNoCollisionVolume, or IntervalCheckNoCollisionEndpoints
	/// indicating if a collision is possible (returns IntervalCheckPossibleCollision); if tetrahedron
	/// (P1, P2, Q1, Q2) has too great a volume for a collision (returns IntervalCheckNoCollisionVolume);
	/// or if the possibly valid collision is not contained
	/// within segments (P1, P2) and (Q1, Q2) (returns IntervalCheckNoCollisionEndpoints).
	IntervalCheckResults possibleCollisionTestNoThickness(const Math::Interval<double>& range) const;

	/// Check if a collision is possible within a specified time interval assuming segments with fixed radius
	/// \param range the parametric [0, 1] time interval over which the collision is to be detected.
	/// \return IntervalCheckPossibleCollision, IntervalCheckNoCollisionVolume, or IntervalCheckNoCollisionEndpoints
	/// indicating if a collision is possible (returns IntervalCheckPossibleCollision); if tetrahedron
	/// (P1, P2, Q1, Q2) has too great a volume for a collision (returns IntervalCheckNoCollisionVolume);
	/// or if the possibly valid collision is not contained
	/// within segments (P1, P2) and (Q1, Q2) (returns IntervalCheckNoCollisionEndpoints).
	IntervalCheckResults possibleCollisionTestWithThickness(const Math::Interval<double>& range) const;

private:
	/// @{
	/// Private constructor and assignment operators to prevent copying.
	SegmentSegmentCcdIntervalCheck(const SegmentSegmentCcdIntervalCheck&);
	SegmentSegmentCcdIntervalCheck& operator=(const SegmentSegmentCcdIntervalCheck&);
	/// @}

	/// @{
	/// Linear motion intervals for each of the segment endpoints from t(0) to t(1).
	Math::LinearMotionND<double, 3>  m_motionP1;
	Math::LinearMotionND<double, 3>  m_motionP2;
	Math::LinearMotionND<double, 3>  m_motionQ1;
	Math::LinearMotionND<double, 3>  m_motionQ2;
	/// @}

	/// @{
	/// Linear motion intervals for relative endpoint differences (i.e. P1Q1 indicates that the
	/// interval encodes Q1 - P1).
	Math::LinearMotionND<double, 3>  m_relativeP1Q1;
	Math::LinearMotionND<double, 3>  m_relativeQ1Q2;
	Math::LinearMotionND<double, 3>  m_relativeP1P2;
	/// @}

	/// The triple product of (Q1(t) - P1(t)) X (P2(t) - P1(t)) X (Q2(t) - Q1(t)) as a 3rd degree polynomial
	/// where P1, P2,Q1 and Q2 are time dependent positions for the segment endpoints.
	/// \note the triple product is equivalent to 6 x the volume of tetrahedron P1P2Q1Q2.
	Math::PolynomialValues<double, 3> m_P1Q1_P1P2_Q1Q2;

	/// @{
	/// Dot product accessors
	/// The dot product for time dependent vertex positions P1(t), P2(t), Q1(t) and Q2(t)
	Math::PolynomialValues<double, 2> m_P1P2_P1Q1;
	Math::PolynomialValues<double, 2> m_Q1Q2_P1Q1;
	Math::PolynomialValues<double, 2> m_P1P2_Q1Q2;
	/// @}

	/// @{
	/// The squared magnitude of the difference operators for time dependent
	/// vertex positions P1(t), P2(t), Q1(t) and Q2(t).
	Math::PolynomialValues<double, 2> m_P1P2_sq;
	Math::PolynomialValues<double, 2> m_Q1Q2_sq;
	/// @}

	/// @{
	/// The x, y and z components of (P2 - P1) X (Q2 - Q1).
	Math::PolynomialValues<double, 2> m_P1P2xQ1Q2_x;
	Math::PolynomialValues<double, 2> m_P1P2xQ1Q2_y;
	Math::PolynomialValues<double, 2> m_P1P2xQ1Q2_z;
	/// @}

	/// @{
	/// The thickness parameters for P and Q, respectively.
	double m_thicknessP;
	double m_thicknessQ;
	/// @}

	/// @{
	/// The algorithm epsilon parameters for "close enough" decisions.
	double m_timePrecisionEpsilon;
	double m_distanceEpsilon;
	double m_volumeEpsilonTimes6;
	double m_muNuEpsilon;
	/// @}
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_SEGMENTSEGMENTINTERVALCCDCHECK_H