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

#ifndef SURGSIM_MATH_CATMULLROM_H
#define SURGSIM_MATH_CATMULLROM_H

#include <vector>

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Math/Vector.h"

/// \file CatmullRom.h collects the utilities to do Catmull-Rom interpolation.
namespace SurgSim
{
namespace Math
{
namespace CatmullRom
{

/// Function to add two 'ghost' points to 'points' at the beginning and the end,
/// prepare 'points' for Catmull-Rom interpolation.
/// \param points List of points to be interpolated.
/// \param[out] result List of points with two ghost points added,
///                    one at the beginning and another at the end of 'points'.
void extendControlPoints(const SurgSim::DataStructures::VerticesPlain& points,
						 std::vector<SurgSim::Math::Vector3d>* result);

/// Run Catmull-Rom interpolation on 'controlPoints'.
/// See https://en.wikipedia.org/wiki/Centripetal_Catmull-Rom_spline for more details.
/// \param subdivisions Number of interpolated points between each pair of control points.
/// \param controlPoints List of points to be interpolated.
/// \param[out] points List of interpolated points, not including the control points.
/// \param tau Defines the tension, affects how sharply the curve bends at the control points.
void interpolate(size_t subdivisions,
				 const std::vector<Math::Vector3d>& controlPoints,
				 std::vector<Math::Vector3d>* points,
				 double tau = 0.4);

}; // namespace CatmullRom
}; // namespace Math
}; // namespace SurgSim

#endif // SURGSIM_MATH_CATMULLROM_H
