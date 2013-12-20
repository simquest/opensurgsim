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

#ifndef SURGSIM_SERIALIZE_MATHCONVERT_H
#define SURGSIM_SERIALIZE_MATHCONVERT_H

#include "SurgSim/Serialize/Convert.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

#include <Eigen/Core>

namespace YAML
{
	/// declaration of specialization convert<SurgSim::Math::Vector4d>
	template <>
	struct convert <SurgSim::Math::Quaterniond>
	{
		static Node encode(const SurgSim::Math::Quaterniond& rhs);
		static bool decode(const Node& node, SurgSim::Math::Quaterniond& rhs);
	};

	/// declaration of specialization convert<SurgSim::Math::RigidTransform3d>
	template <>
	struct convert <SurgSim::Math::RigidTransform3d>
	{
		static Node encode(const SurgSim::Math::RigidTransform3d& rhs);
		static bool decode(const Node& node, SurgSim::Math::RigidTransform3d& rhs);
	};

	template <class Type, int Rows, int Cols, int MOpt>
	struct convert <typename Eigen::Matrix<Type, Rows, Cols, MOpt>>
	{
		static Node convert<typename Eigen::Matrix<Type, Rows, Cols, MOpt>>::encode(
			const typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs);
		static bool convert<typename Eigen::Matrix<Type, Rows, Cols, MOpt>>::decode(
			const Node& node,
			typename Eigen::Matrix<Type, Rows, Cols, MOpt>& rhs);
	};

	/// Specialization for Eigen Row Vectors, which are the type that Vector2x, Vector3x use
	template <class Type, int Rows, int MOpt>
	struct convert <typename Eigen::Matrix<Type,Rows,1,MOpt>>
	{
		static Node convert<typename Eigen::Matrix<Type, Rows, 1, MOpt>>::encode(
			const typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs);
		static bool convert<typename Eigen::Matrix<Type, Rows, 1, MOpt>>::decode(
			const Node& node,
			typename Eigen::Matrix<Type, Rows, 1, MOpt>& rhs);
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

#include "SurgSim/Serialize/MathConvert-inl.h"

#endif // SURGSIM_SERIALIZE_MATHCONVERT_H