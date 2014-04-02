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

/// 	http://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?page=3
/// Another alternative is probably to use !a.intersection(b).isEmpty() as a test ...
template <class Scalar, int Dim>
bool doAabbIntersect(const Eigen::AlignedBox<Scalar, Dim>& a,
					 const Eigen::AlignedBox<Scalar, Dim>& b,
					 double tolerance)
{
	typedef typename Eigen::AlignedBox<Scalar, Dim>::VectorType VectorType;

	VectorType vector = (b.center() - a.center()).array().abs();
	VectorType totalSizes = ((a.sizes() + b.sizes()) * 0.5).array() + tolerance;

	return (vector.array() <= totalSizes.array()).all();
}

template <class Scalar, int Dim>
bool doAabbIntersect(const Eigen::AlignedBox<Scalar, Dim>& a,
					 const Eigen::AlignedBox<Scalar, Dim>& b)
{
	return !a.intersection(b).isEmpty();
}

}
}

#endif
