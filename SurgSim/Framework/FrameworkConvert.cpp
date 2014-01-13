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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Component.h"



namespace YAML
{
	Node convert<std::shared_ptr<SurgSim::Framework::Component>>::encode(const std::shared_ptr<SurgSim::Framework::Component> rhs)
	{
		Node result;
		result["id"] = rhs->getId();
		result["className"] = rhs->getClassName();
		result["name"] = rhs->getName();
		return result;
	}

	bool convert<std::shared_ptr<SurgSim::Framework::Component>>::decode(const Node& node,
		std::shared_ptr<SurgSim::Framework::Component>& rhs)
	{
		bool result = false;
		if (node.IsMap() && node["id"].IsDefined() && node["className"].IsDefined() && node["name"].IsDefined())
		{
			if (rhs == nullptr)
			{
				std::string id = node["id"].as<std::string>();
				RegistryType& registry = getRegistry();
				auto sharedComponent = registry.find(id);
				if ( sharedComponent != registry.end())
				{
					rhs = sharedComponent->second;
				}
				else
				{
					std::string className = node["className"].as<std::string>();
					SurgSim::Framework::Component::FactoryType& factory = 
						SurgSim::Framework::Component::getFactory();

					if (factory.isRegistered(className))
					{
						rhs = factory.create(node["className"].as<std::string>(),node["name"].as<std::string>());
						getRegistry()[id] = rhs;
					}
					else
					{
						SURGSIM_FAILURE() << "Class " << className << " is not registered in the factory.";
					}
				}
			}
			rhs->decode(node);
			result = true;
		}
		return result;
	}

	convert<std::shared_ptr<SurgSim::Framework::Component>>::RegistryType&
		convert<std::shared_ptr<SurgSim::Framework::Component>>::getRegistry()
	{
		static RegistryType registry;
		return registry;
	}

	Node convert<SurgSim::Framework::Component>::encode(const SurgSim::Framework::Component& rhs)
	{

		YAML::Node node(rhs.encode());
		node["id"] = rhs.getId();
		node["className"] = rhs.getClassName();
		node["name"] = rhs.getName();
		return node;
	}

}