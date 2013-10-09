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

#ifndef SURGSIM_SERIALIZE_GRAPHICSCONVERT_INL_H
#define SURGSIM_SERIALIZE_GRAPHICSCONVERT_INL_H

#include <SurgSim/Serialize/FrameworkConvert.h>
#include <SurgSim/Graphics/SphereRepresentation.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>

namespace YAML
{
	/// Specialize of YAML::convert<> template Presensation class.
	template <>
	struct convert <SurgSim::Graphics::Representation>
	{
		static Node encode(const SurgSim::Graphics::Representation& rhs)
		{
			Node node;

			node = convert<SurgSim::Framework::Component>::encode(rhs);
			node["initialPose"] = rhs.getInitialPose();
			node["pose"] = rhs.getPose();

			return node;
		}

		static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::Representation> rhs)
		{
			if (! node.IsMap())
			{
				return false;
			}
			convert<SurgSim::Framework::Component>::decode(node, rhs);
			rhs->setInitialPose(node["initialPose"].as<SurgSim::Math::RigidTransform3d>());
			rhs->setPose(node["pose"].as<SurgSim::Math::RigidTransform3d>());
			return true;
		}

	};

	/// Specialize of YAML::convert<> template SpherePresensation class.
	template <>
	struct convert <SurgSim::Graphics::SphereRepresentation>
	{
		static Node encode(const SurgSim::Graphics::SphereRepresentation& rhs)
		{
			Node node;

			node = convert<SurgSim::Graphics::Representation>::encode(rhs);
			node["class"] = "SphereRepresentation";
			node["radius"] = rhs.getRadius();
			return node;
		}

		static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs)
		{
			if (! node.IsMap())
			{
				return false;
			}
			convert<SurgSim::Graphics::Representation>::decode(node, rhs);
			rhs->setRadius(node["radius"].as<double>());
			return true;
		}
	};

}

#endif // SURGSIM_SERIALIZE_OBJECTCONVERTER_INL_H
