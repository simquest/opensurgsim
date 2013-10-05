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

#ifndef SURGSIM_SERIALIZE_OBJECTCONVERTER_INL_H
#define SURGSIM_SERIALIZE_OBJECTCONVERTER_INL_H

namespace YAML
{

	/// Specialize of YAML::convert<> template SpherePresensation class.
	template <>
	struct convert <SurgSim::Graphics::SphereRepresentation>
	{
		static Node encode(const std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs)
		{
			Node node;
			node["name"] = "SphereRepresentation";
			node["radius"] = rhs->getRadius();
			node["initialPose"] = rhs->getInitialPose();
			node["pose"] = rhs->getPose();
			return node;
		}

		static bool decode(const Node& node, std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs)
		{		
			if (! node.IsMap())
			{
				return false;
			}

			rhs->setRadius(node["radius"].as<double>());
			rhs->setInitialPose(node["initialPose"].as<SurgSim::Math::RigidTransform3d>());
			rhs->setPose(node["pose"].as<SurgSim::Math::RigidTransform3d>());
			return true;
		}
	};

	void exportComponent(Emitter& out, const std::shared_ptr<SurgSim::Framework::Component> rhs)
	{
		out << YAML::Key << "name" << YAML::Value << rhs->getName();
	}

	void exportRepresentation(Emitter& out, const std::shared_ptr<SurgSim::Graphics::Representation> rhs)
	{
		out << YAML::Key << "initialPose" << YAML::Value << rhs->getInitialPose();
		out << YAML::Key << "pose" << YAML::Value << rhs->getPose();
		exportComponent(out, rhs);
	}

	void exportSphereRepresentation(Emitter& out, const std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs)
	{
		out << YAML::Key << "radius" << YAML::Value << rhs->getRadius();
		exportRepresentation(out, rhs);
	}

	// Overload << for YAML::Emitter to support SurgSim::Graphics::SpherePresentation type
	Emitter& operator << (Emitter& out, const std::shared_ptr<SurgSim::Graphics::SphereRepresentation> rhs)
	{  
		out << YAML::BeginMap;
		out << YAML::Key << "class" << YAML::Value << "SphereRepresentation";
		exportSphereRepresentation(out, rhs);
		out << YAML::EndMap;
		return out;
	}
}

#endif // SURGSIM_SERIALIZE_OBJECTCONVERTER_INL_H
