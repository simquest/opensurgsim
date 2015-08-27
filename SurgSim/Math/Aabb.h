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

#ifndef SURGSIM_MATH_AABB_H
#define SURGSIM_MATH_AABB_H

#include <Eigen/Geometry>

namespace SurgSim
{
namespace Math
{

/// Wrapper around the Eigen type
typedef Eigen::AlignedBox<float, 3> Aabbf;

/// Wrapper around the Eigen type
typedef Eigen::AlignedBox<double, 3> Aabbd;

/// Determine whether two AABBs have an intersection with each other, for the calculation see
/// http://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=3
/// \tparam Scalar numeric type
/// \tparam Dim dimension of the space to be used
/// \param aabb0 first axis aligned bounding box
/// \param aabb1 second axis aligned bounding box
/// \param tolerance the bounding boxes will be considered bigger by this amount
/// \return true if there is an overlap between the two boxes
template <class Scalar, int Dim>
bool doAabbIntersect(const Eigen::AlignedBox<Scalar, Dim>& aabb0,
					 const Eigen::AlignedBox<Scalar, Dim>& aabb1,
					 double tolerance)
{
	typedef typename Eigen::AlignedBox<Scalar, Dim>::VectorType VectorType;

	VectorType vector = (aabb1.center() - aabb0.center()).array().abs();
	VectorType totalSizes = ((aabb0.sizes() + aabb1.sizes()) * 0.5).array() + tolerance;

	return (vector.array() <= totalSizes.array()).all();
}

/// Determine whether two AABBs overlap, using a minimal set of eigen calls, does not take a tolerance
/// \tparam Scalar numeric type
/// \tparam Dim dimension of the space to be used
/// \param a first axis aligned bounding box
/// \param b second axis aligned bounding box
/// \return true if there is an overlap between the two boxes
template <class Scalar, int Dim>
bool doAabbIntersect(const Eigen::AlignedBox<Scalar, Dim>& a,
					 const Eigen::AlignedBox<Scalar, Dim>& b)
{
	return !a.intersection(b).isEmpty();
}

/// Convenience function for creating a bounding box from three vertices (e.g. the vertices of a triangle)
/// \tparam Scalar numeric type
/// \tparam Dim dimension of the space to be used
/// \tparam MType the eigen type of the vectors
/// \return an AABB containing all the points passed
template <class Scalar, int Dim, int MType>
Eigen::AlignedBox<Scalar, Dim> makeAabb(
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector0,
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector1,
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector2)
{
	Eigen::AlignedBox<Scalar, Dim> result(vector0);
	result.extend(vector1);
	result.extend(vector2);
	return std::move(result);
}

/// Convenience function for creating a bounding box from four vertices
/// \tparam Scalar numeric type
/// \tparam Dim dimension of the space to be used
/// \tparam MType the eigen type of the vectors
/// \return an AABB containing all the points passed
template <class Scalar, int Dim, int MType>
Eigen::AlignedBox<Scalar, Dim> makeAabb(
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector0,
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector1,
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector2,
	const Eigen::Matrix<Scalar, Dim, 1, MType>& vector3)
{
	Eigen::AlignedBox<Scalar, Dim> result(vector0);
	result.extend(vector1);
	result.extend(vector2);
	result.extend(vector3);
	return std::move(result);
}

}
}

#endif
