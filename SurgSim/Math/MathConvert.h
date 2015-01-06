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

#ifndef SURGSIM_MATH_MATHCONVERT_H
#define SURGSIM_MATH_MATHCONVERT_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/OdeSolver.h"

namespace SurgSim
{
namespace Math
{
class Shape;
}
}

/// \file MathConvert.h
/// This contains a series of functions to encode and decode Eigen data structures to
/// and from YAML nodes. These conversion functions will extinguish Eigen options, these
/// are not serialized, the output is determined by the type as it is declared in the
/// appropriate conversion function, with Eigen::Transform, this could lead to problems
/// If the mode that is used for reading is different than the mode that was used while
/// writing.

namespace YAML
{

/// Specialization of convert for fixed size Eigen::Matrix
SURGSIM_DOUBLE_SPECIALIZATION
template <typename Type, int Rows, int Cols, int MOpt>
struct convert<typename Eigen::Matrix<Type, Rows, Cols, MOpt>>
{
	static Node encode(const typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs);
	static bool decode(const Node& node, typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs); //NOLINT
};

/// Specialization for Eigen Row Vectors, which are the type that Vector2x, Vector3x use
SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Rows, int MOpt>
struct convert <typename Eigen::Matrix<Type,Rows,1,MOpt>>
{
	static Node encode(const typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs);
	static bool decode(const Node& node, typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs); //NOLINT
};

/// Specialization of convert for Eigen::Quaternion
SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int QOpt>
struct convert<typename Eigen::Quaternion<Type, QOpt>>
{
	static Node encode(const typename Eigen::Quaternion<Type, QOpt>& rhs);
	static bool decode(const Node& node, typename Eigen::Quaternion<Type, QOpt>& rhs); //NOLINT
};

/// Specialization of convert for Eigen::RigidTransform
SURGSIM_DOUBLE_SPECIALIZATION
template <class Type, int Dim, int TMode, int TOptions>
struct convert<typename Eigen::Transform<Type, Dim, TMode, TOptions>>
{
	static Node encode(const typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs);
	static bool decode(const Node& node, typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs); //NOLINT
};

template <>
struct convert<std::shared_ptr<SurgSim::Math::Shape>>
{
	static Node encode(const std::shared_ptr<SurgSim::Math::Shape>& rhs);
	static bool decode(const Node& node, std::shared_ptr<SurgSim::Math::Shape>& rhs); //NOLINT
};

template <>
struct convert<SurgSim::Math::IntegrationScheme>
{
	static Node encode(const SurgSim::Math::IntegrationScheme& rhs);
	static bool decode(const Node& node, SurgSim::Math::IntegrationScheme& rhs); //NOLINT
};

};

#include "SurgSim/Math/MathConvert-inl.h"

#endif // SURGSIM_MATH_MATHCONVERT_H
