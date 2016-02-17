// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_MATH_POINTTRIANGLECCDCONTACTCALCULATION_INL_H
#define SURGSIM_MATH_POINTTRIANGLECCDCONTACTCALCULATION_INL_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SurgSim/Math/CubicSolver.h"

namespace SurgSim
{
namespace Math
{

/// Check if a point belongs to a triangle at a given time of their motion
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param time The time of coplanarity of the 4 points (P(t), A(t), B(t), C(t) are expected to be coplanar)
/// \param P the point motion (from first to second)
/// \param A, B, C The triangle points motion (from first to second)
/// \param[out] barycentricCoordinates The barycentric coordinates of P(t) in ABC(t)
/// \return true if P(t) is inside the triangle ABC(t)
template <class T, int MOpt>
bool isPointInsideTriangle(
	T time,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& P,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& A,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& B,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& C,
	Eigen::Matrix<T, 3, 1, MOpt>* barycentricCoordinates)
{
	Eigen::Matrix<T, 3, 1, MOpt> Pt = interpolate(P.first, P.second, time);
	Eigen::Matrix<T, 3, 1, MOpt> At = interpolate(A.first, A.second, time);
	Eigen::Matrix<T, 3, 1, MOpt> Bt = interpolate(B.first, B.second, time);
	Eigen::Matrix<T, 3, 1, MOpt> Ct = interpolate(C.first, C.second, time);

	bool result = Math::barycentricCoordinates(Pt, At, Bt, Ct, barycentricCoordinates);

	for (int i = 0; i < 3; i++)
	{
		if (std::abs((*barycentricCoordinates)[i]) < Math::Geometry::ScalarEpsilon)
		{
			(*barycentricCoordinates)[i] = 0.0;
		}
		if (std::abs(1.0 - (*barycentricCoordinates)[i]) < Math::Geometry::ScalarEpsilon)
		{
			(*barycentricCoordinates)[i] = 1.0;
		}
	}

	return (result &&
		(*barycentricCoordinates)[0] >= 0.0 &&
		(*barycentricCoordinates)[1] >= 0.0 &&
		(*barycentricCoordinates)[2] >= 0.0);
}

/// Continuous collision detection between a point P and a triangle ABC
/// \tparam T		Accuracy of the calculation, can usually be inferred.
/// \tparam MOpt	Eigen Matrix options, can usually be inferred.
/// \param P the point motion (from first to second) to calculate the ccd with
/// \param A, B, C The triangle points motion (from first to second) to calculate the ccd with
/// \param[out] timeOfImpact The time of impact within [0..1] if a collision is found
/// \param[out] tv01Param, tv02Param The barycentric coordinate of the contact point in the triangle
/// i.e. ContactPoint(timeOfImpact) = A(timeOfImpact) + tv01Param.AB(timeOfImpact) + tv02Param.AC(timeOfImpact)
/// \return True if the given point/triangle motions intersect, False otherwise
/// \note Simple cubic-solver https://graphics.stanford.edu/courses/cs468-02-winter/Papers/Collisions_vetements.pdf
/// \note Optimized method http://www.robotics.sei.ecnu.edu.cn/CCDLY/GMOD15.pdf
/// \note Optimized method https://www.cs.ubc.ca/~rbridson/docs/brochu-siggraph2012-ccd.pdf
template <class T, int MOpt> inline
bool calculateCcdContactPointTriangle(
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& P,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& A,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& B,
	const std::pair<Eigen::Matrix<T, 3, 1, MOpt>, Eigen::Matrix<T, 3, 1, MOpt>>& C,
	T* timeOfImpact, T* tv01Param, T* tv02Param)
{
	std::array<T, 3> roots;
	int numberOfRoots = timesOfCoplanarityInRange01(P, A, B, C, &roots);

	// The roots are all in [0..1] and ordered ascendingly
	for (int rootId = 0; rootId < numberOfRoots; ++rootId)
	{
		Eigen::Matrix<T, 3, 1, MOpt> baryCoords;
		if (isPointInsideTriangle(roots[rootId], P, A, B, C, &baryCoords))
		{
			// The point P is in the triangle plane at time t, and is inside the triangle
			*timeOfImpact = roots[rootId];
			*tv01Param = baryCoords[1];
			*tv02Param = baryCoords[2];

			// None of these assertion should be necessary, but just to double check
			SURGSIM_ASSERT(*timeOfImpact >= 0.0 && *timeOfImpact <= 1.0);
			SURGSIM_ASSERT(*tv01Param >= 0.0);
			SURGSIM_ASSERT(*tv02Param >= 0.0);
			SURGSIM_ASSERT(*tv01Param + *tv02Param <= 1.0 + Geometry::ScalarEpsilon);

			return true;
		}
	}

	return false;
}

}; // namespace Math
}; // namespace SurgSim

#endif // SURGSIM_MATH_POINTTRIANGLECCDCONTACTCALCULATION_INL_H
