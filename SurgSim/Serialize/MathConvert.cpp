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

#include "SurgSim/Serialize/MathConvert.h"

namespace YAML
{

	/// Specialize of YAML::convert<> template RigidTransform class.

	/// Overload << for YAML::Emitter to support SurgSim::Math::Vector3d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Vector3d& rhs)
	{
		out << Flow;
		out << BeginSeq << rhs[0] << rhs[1] << rhs[2] << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Vector4d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Vector4d& rhs)
	{
		out << Flow;
		out << BeginSeq << rhs[0] << rhs[1] << rhs[2] << rhs[3] << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Quaterniond type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Quaterniond& rhs)
	{
		return (out << rhs.coeffs());
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Matrix33d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Matrix33d& rhs)
	{
		out << Flow;
		out << BeginSeq;
		for (auto row = 0; row < rhs.rows(); ++row)
		{
			out << static_cast<SurgSim::Math::Vector3d>(rhs.row(row));
		}
		out << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::Matrix44d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::Matrix44d& rhs)
	{
		out << Flow;
		out << BeginSeq;
		for (auto row = 0; row < rhs.rows(); ++row)
		{
			out << static_cast<SurgSim::Math::Vector4d>(rhs.row(row));
		}
		out << EndSeq;
		return out;
	}

	/// Overload << for YAML::Emitter to support SurgSim::Math::RigidTransform3d type
	Emitter& operator << (Emitter& out, const SurgSim::Math::RigidTransform3d& rhs)
	{
		SurgSim::Math::Matrix44d transform = rhs.matrix();
		return (out << transform);
	}

}