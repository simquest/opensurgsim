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

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Scene.h"

#include <vector>

#include <boost/uuid/uuid_io.hpp>

namespace
{
const std::string ClassNamePropertyName = "ClassName";
const std::string NamePropertyName = "Name";
const std::string IdPropertyName = "Id";
}

namespace YAML
{
Node convert<std::shared_ptr<SurgSim::Framework::Component>>::encode(
			const std::shared_ptr<SurgSim::Framework::Component> rhs)
{
	Node result;
	result[IdPropertyName] = to_string(rhs->getUuid());
	result[ClassNamePropertyName] = rhs->getClassName();
	result[NamePropertyName] = rhs->getName();
	return result;
}

bool convert<std::shared_ptr<SurgSim::Framework::Component>>::decode(const Node& node,
		std::shared_ptr<SurgSim::Framework::Component>& rhs)
{
	bool result = false;
	if (node.IsMap() &&
		node[IdPropertyName].IsDefined() &&
		node[ClassNamePropertyName].IsDefined() &&
		node[NamePropertyName].IsDefined())
	{
		if (rhs == nullptr)
		{
			std::string id = node[IdPropertyName].as<std::string>();
			RegistryType& registry = getRegistry();
			auto sharedComponent = registry.find(id);
			if (sharedComponent != registry.end())
			{
				SURGSIM_ASSERT(node[NamePropertyName].as<std::string>() == sharedComponent->second->getName() &&
					node[ClassNamePropertyName].as<std::string>() == sharedComponent->second->getClassName()) <<
							"The current node: " << std::endl << node << "has the same id as an instance " <<
							"already registered, but the name and/or the className are different. This is " <<
							"likely a problem with a manually assigned id.";
				rhs = sharedComponent->second;
			}
			else
			{
				std::string className = node[ClassNamePropertyName].as<std::string>();
				SurgSim::Framework::Component::FactoryType& factory =
					SurgSim::Framework::Component::getFactory();

				if (factory.isRegistered(className))
				{
					rhs = factory.create(
							  node[ClassNamePropertyName].as<std::string>(),
							  node[NamePropertyName].as<std::string>());
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
	node[IdPropertyName] = to_string(rhs.getUuid());
	node[ClassNamePropertyName] = rhs.getClassName();
	node[NamePropertyName] = rhs.getName();
	return node;
}


Node convert<std::shared_ptr<SurgSim::Framework::SceneElement>>::encode(
			const std::shared_ptr<SurgSim::Framework::SceneElement> rhs)
{
	SURGSIM_ASSERT(rhs != nullptr) << "Trying to encode nullptr SceneElement";
	return rhs->encode(false);
}

bool convert<std::shared_ptr<SurgSim::Framework::SceneElement>>::decode(
			const Node& node,
			std::shared_ptr<SurgSim::Framework::SceneElement>& rhs)
{
	if (rhs == nullptr)
	{
		// For now only deal with BasicSceneElement classes
		rhs = std::make_shared<SurgSim::Framework::BasicSceneElement>("");
	}
	return rhs->decode(node);
}

Node convert<SurgSim::Framework::SceneElement>::encode(
	const SurgSim::Framework::SceneElement& rhs)
{
	return rhs.encode(true);
}

Node convert<std::shared_ptr<SurgSim::Framework::Scene>>::encode(
			const std::shared_ptr<SurgSim::Framework::Scene> rhs)
{
	SURGSIM_ASSERT(rhs != nullptr) << "Trying to encode nullptr Scene";
	return rhs->encode();
}

bool convert<std::shared_ptr<SurgSim::Framework::Scene>>::decode(
			const Node& node,
			std::shared_ptr<SurgSim::Framework::Scene>& rhs)
{
	bool result = false;
	if (rhs != nullptr)
	{
		result = rhs->decode(node);
	}
	return result;
}

}