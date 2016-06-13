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

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Asset.h"

#include <boost/uuid/uuid_io.hpp>

namespace
{
const std::string NamePropertyName = "Name";
const std::string IdPropertyName = "Id";
}

namespace YAML
{
Node convert<std::shared_ptr<SurgSim::Framework::Component>>::encode(
			const std::shared_ptr<SurgSim::Framework::Component> rhs)
{
	Node result;
	if (nullptr != rhs)
	{
		Node data;
		data[IdPropertyName] = to_string(rhs->getUuid());
		data[NamePropertyName] = rhs->getName();
		result[rhs->getClassName()] = data;
	}
	return result;
}

bool convert<std::shared_ptr<SurgSim::Framework::Component>>::decode(
			const Node& node,
			std::shared_ptr<SurgSim::Framework::Component>& rhs) //NOLINT
{
	bool result = false;

	if (!node.IsMap())
	{
		return false;
	}

	SurgSim::Framework::Component::FactoryType& factory = SurgSim::Framework::Component::getFactory();
	std::string className = node.begin()->first.as<std::string>();
	SURGSIM_ASSERT(factory.isRegistered(className)) << "Class " << className << " is not registered in the factory.";

	Node data = node.begin()->second;
	if (data.IsMap() && data[NamePropertyName].IsDefined())
	{
		std::string name = data[NamePropertyName].as<std::string>();
		if (rhs == nullptr)
		{
			if (data[IdPropertyName].IsDefined())
			{
				std::string id = data[IdPropertyName].as<std::string>();
				RegistryType& registry = getRegistry();
				auto sharedComponent = registry.find(id);
				if (sharedComponent != registry.end())
				{
					SURGSIM_ASSERT(name == sharedComponent->second->getName() &&
								   className == sharedComponent->second->getClassName())
							<< "The current node:" << std::endl << node <<  std::endl << "has the same id as an "
							<< "instance already registered, but the name and/or the className are different. This is "
							<< "likely a problem with a manually assigned id." << std::endl
							<< "The original component is a:" << std::endl
							<< sharedComponent->second->getClassName() << std::endl
							<< sharedComponent->second->getName();
					rhs = sharedComponent->second;
				}
				else
				{
					rhs = factory.create(className, name);
					getRegistry()[id] = rhs;
				}
			}
			else
			{
				rhs = factory.create(className, name);
			}
		}

		std::vector<std::string> ignoredProperties;
		ignoredProperties.push_back(NamePropertyName);
		ignoredProperties.push_back(IdPropertyName);

		rhs->decode(data, ignoredProperties);
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
	YAML::Node data(rhs.encode());
	data[IdPropertyName] = to_string(rhs.getUuid());
	data[NamePropertyName] = rhs.getName();

	YAML::Node result;
	result[rhs.getClassName()] = data;

	return result;
}


Node convert<std::shared_ptr<SurgSim::Framework::SceneElement>>::encode(
			const std::shared_ptr<SurgSim::Framework::SceneElement> rhs)
{
	SURGSIM_ASSERT(rhs != nullptr) << "Trying to encode nullptr SceneElement";
	return rhs->encode(false);
}

bool convert<std::shared_ptr<SurgSim::Framework::SceneElement>>::decode(
			const Node& node,
			std::shared_ptr<SurgSim::Framework::SceneElement>& rhs) //NOLINT
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
			std::shared_ptr<SurgSim::Framework::Scene>& rhs) //NOLINT
{
	bool result = false;
	if (rhs != nullptr)
	{
		result = rhs->decode(node);
	}
	return result;
}

YAML::Node YAML::convert<std::shared_ptr<SurgSim::Framework::Asset>>::encode(
			const std::shared_ptr<SurgSim::Framework::Asset> rhs)
{
	YAML::Node node;
	node[rhs->getClassName()] = rhs->encode();
	return node;
}

bool YAML::convert<std::shared_ptr<SurgSim::Framework::Asset>>::decode(
			const Node& node, std::shared_ptr<SurgSim::Framework::Asset>& rhs) //NOLINT
{
	bool result = false;

	if (node.IsMap())
	{
		if (nullptr == rhs)
		{
			std::string className = node.begin()->first.as<std::string>();
			auto& factory = SurgSim::Framework::Asset::getFactory();

			if (factory.isRegistered(className))
			{
				rhs = factory.create(className);
			}
			else
			{
				SURGSIM_FAILURE() << "Class " << className << " is not registered in the Asset factory.";
			}
		}

		Node data = node.begin()->second;
		if (data.IsMap())
		{
			rhs->decode(data);
		}

		result = true;
	}

	return result;
}

}
