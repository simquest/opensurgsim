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

#ifndef SURGSIM_FRAMEWORK_MATHCONVERT_H
#define SURGSIM_FRAMEWORK_MATHCONVERT_H

#include "SurgSim/Framework/Convert.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

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
		static bool decode(const Node& node, typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs);
	};

	/// Specialization for Eigen Row Vectors, which are the type that Vector2x, Vector3x use
	SURGSIM_DOUBLE_SPECIALIZATION
	template <class Type, int Rows, int MOpt>
	struct convert <typename Eigen::Matrix<Type,Rows,1,MOpt>>
	{
		static Node encode(const typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs);
		static bool decode(const Node& node, typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs);
	};

	/// Specialization of convert for Eigen::Quaternion
	SURGSIM_DOUBLE_SPECIALIZATION
	template <class Type, int QOpt>
	struct convert<typename Eigen::Quaternion<Type, QOpt>>
	{
		static Node encode(const typename Eigen::Quaternion<Type, QOpt>& rhs);
		static bool decode(const Node& node, typename Eigen::Quaternion<Type, QOpt>& rhs);
	};

	/// Specialization of convert for Eigen::RigidTransform
	SURGSIM_DOUBLE_SPECIALIZATION
	template <class Type, int Dim, int TMode, int TOptions>
	struct convert<typename Eigen::Transform<Type, Dim, TMode, TOptions>>
	{
		static Node encode(const typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs);
		static bool decode(const Node& node, typename Eigen::Transform<Type, Dim, TMode, TOptions>& rhs);
	};



	// Overload << for YAML::Emitter to support SurgSim::Math::Vector3d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Vector3d& rhs);

	// Overload << for YAML::Emitter to support SurgSim::Math::Vector4d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Vector4d& rhs);

	// Overload << for YAML::Emitter to support SurgSim::Math::Quaterniond type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Quaterniond& rhs);

	// Overload << for YAML::Emitter to support SurgSim::Math::Matrix33d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Matrix33d& rhs);

	// Overload << for YAML::Emitter to support SurgSim::Math::Matrix44d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Matrix44d& rhs);

	// Overload << for YAML::Emitter to support SurgSim::Math::RigidTransform3d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::RigidTransform3d& rhs);
};

#include "SurgSim/Math/MathConvert-inl.h"

#endif // SURGSIM_FRAMEWORK_MATHCONVERT_H