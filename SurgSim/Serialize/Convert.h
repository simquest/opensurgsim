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

#ifndef SURGSIM_SERIALIZE_CONVERT_H
#define SURGSIM_SERIALIZE_CONVERT_H

#include <yaml-cpp/yaml.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/SphereRepresentation.h>
#include <SurgSim/Framework/Log.h>

namespace SurgSim
{
namespace Serialize
{
	/// Logger name for Serialization
	const std::string serializeLogger = "Serialization";
};
};

namespace YAML
{
	/// declaration of specialization convert<SurgSim::Math::Vector3d>
	template <>
	struct convert <SurgSim::Math::Vector3d>
	{
		static Node encode(const SurgSim::Math::Vector3d& rhs);
		static bool decode(const Node &node, SurgSim::Math::Vector3d& rhs);
	};

	/// declaration of specialization convert<SurgSim::Math::Vector4d>
	template <>
	struct convert <SurgSim::Math::Vector4d>
	{
		static Node encode(const SurgSim::Math::Vector4d& rhs);
		static bool decode(const Node& node, SurgSim::Math::Vector4d& rhs);
	};

	/// declaration of specialization convert<SurgSim::Math::Vector4d>
	template <>
	struct convert <SurgSim::Math::Quaterniond>
	{
		static Node encode(const SurgSim::Math::Quaterniond& rhs);
		static bool decode(const Node& node, SurgSim::Math::Quaterniond& rhs);
	};

	/// declaration of specialization convert<SurgSim::Math::Matrix33d>
	template <>
	struct convert <SurgSim::Math::Matrix33d>
	{
		static Node encode(const SurgSim::Math::Matrix33d& rhs);
		static bool decode(const Node& node, SurgSim::Math::Matrix33d& rhs);
	};

	/// declaration of specialization convert<SurgSim::Math::Matrix44d>
	template <>
	struct convert <SurgSim::Math::Matrix44d>
	{
		static Node encode(const SurgSim::Math::Matrix44d& rhs);
		static bool decode(const Node& node, SurgSim::Math::Matrix44d& rhs);
	};

	/// declaration of specialization convert<SurgSim::Math::RigidTransform3d>
	template <>
	struct convert <SurgSim::Math::RigidTransform3d>
	{
		static Node encode(const SurgSim::Math::RigidTransform3d& rhs);
		static bool decode(const Node& node, SurgSim::Math::RigidTransform3d& rhs);
	};

	/// Specialize of YAML::convert<> template Component class.
	template <>
	struct convert <SurgSim::Framework::Component>
	{
		static Node encode(const SurgSim::Framework::Component& rhs);
		static bool decode(const Node& node, std::shared_ptr<SurgSim::Framework::Component> rhs);
	};

	/// Specialize of YAML::convert<> template Presensation class.
	template <>
	struct convert <SurgSim::Graphics::Representation>
	{
		static Node encode(const SurgSim::Graphics::Representation& rhs);
		static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::Representation> rhs);

	};

	/// Specialize of YAML::convert<> template SpherePresensation class.
	template <>
	struct convert <SurgSim::Graphics::SphereRepresentation>
	{
		static Node encode(const SurgSim::Graphics::SphereRepresentation& rhs);
		static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs);
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

#endif // SURGSIM_SERIALIZE_CONVERT_H