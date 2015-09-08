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

#include <algorithm>
#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{
namespace Framework
{

Accessible::Accessible()
{

}

Accessible::~Accessible()
{

}

template <>
boost::any Accessible::getValue(const std::string& name) const
{
	auto functors = m_functors.find(name);
	if (functors != std::end(m_functors) && functors->second.getter != nullptr)
	{
		return functors->second.getter();
	}
	else
	{
		SURGSIM_FAILURE() << "Can't get property: " << name << ". " << ((functors == std::end(m_functors)) ?
						  "Property not found." : "No getter defined for property.");
		return boost::any();
	}
}

boost::any Accessible::getValue(const std::string& name) const
{
	return getValue<boost::any>(name);
}


void Accessible::setValue(const std::string& name, const boost::any& value)

{
	auto functors = m_functors.find(name);
	if (functors != std::end(m_functors) && functors->second.setter != nullptr)
	{
		functors->second.setter(value);
	}
	else
	{
		SURGSIM_FAILURE() << "Can't set property: " << name << ". " << ((functors == std::end(m_functors)) ?
						  "Property not found." : "No setter defined for property.");
	}
}


void Accessible::setGetter(const std::string& name, GetterType func)
{
	SURGSIM_ASSERT(func != nullptr) << "Getter functor can't be nullptr";
	m_functors[name].getter = func;
}

void Accessible::setSetter(const std::string& name, SetterType func)
{
	SURGSIM_ASSERT(func != nullptr) << "Setter functor can't be nullptr";
	m_functors[name].setter = func;
}

void Accessible::setAccessors(const std::string& name, GetterType getter, SetterType setter)
{
	setGetter(name, getter);
	setSetter(name, setter);
}


void Accessible::removeAccessors(const std::string& name)
{
	auto functors = m_functors.find(name);
	if (functors != std::end(m_functors))
	{
		functors->second.setter = nullptr;
		functors->second.getter = nullptr;
	}
}


bool Accessible::isReadable(const std::string& name) const
{
	auto functors = m_functors.find(name);
	return (functors != m_functors.end() && functors->second.getter != nullptr);
}

bool Accessible::isWriteable(const std::string& name) const
{
	auto functors = m_functors.find(name);
	return (functors != m_functors.end() && functors->second.setter != nullptr);
}

void Accessible::setSerializable(const std::string& name, EncoderType encoder, DecoderType decoder)
{
	SURGSIM_ASSERT(encoder != nullptr) << "Encoder functor can't be nullptr.";
	SURGSIM_ASSERT(decoder != nullptr) << "Decoder functor can't be nullptr.";

	m_functors[name].encoder = encoder;
	m_functors[name].decoder = decoder;
}

void Accessible::setDecoder(const std::string& name, DecoderType decoder)
{
	SURGSIM_ASSERT(decoder != nullptr) << "Decoder functor can't be nullptr";

	m_functors[name].decoder = decoder;
}

YAML::Node Accessible::encode() const
{
	YAML::Node result;
	for (auto functors = m_functors.cbegin(); functors != m_functors.cend(); ++functors)
	{
		auto encoder = functors->second.encoder;
		if (encoder != nullptr)
		{
			result[functors->first] = encoder();
		}
	}
	return result;
}

void Accessible::decode(const YAML::Node& node, const std::vector<std::string>& ignoredProperties)
{
	SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("Framework/Accessible")) <<
		"Decoding node: \n" << node;
	SURGSIM_ASSERT(node.IsMap()) << "Node to be decoded has to be map.";

	for (auto data = node.begin(); data != node.end(); ++data)
	{
		std::string name = data->first.as<std::string>();
		if (std::find(std::begin(ignoredProperties), std::end(ignoredProperties), name) != std::end(ignoredProperties))
		{
			continue;
		}

		auto functors = m_functors.find(name);
		if (functors == std::end(m_functors) || !functors->second.decoder)
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Framework/Accessible"))
					<< "Can't find property with name " << name << " in the accessible, while trying to set " <<
					data->second;
		}
		else
		{
			YAML::Node temporary = data->second;
			if (!temporary.IsNull() && temporary.IsDefined())
			{
				try
				{
					functors->second.decoder(&temporary);
				}
				catch (std::exception e)
				{
					SURGSIM_FAILURE() << e.what() << " for value " << temporary.as<std::string>();
				}
			}
			else
			{
				SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getLogger("Framework/Accessible"))
						<< "Found property with name " << name << " in the accessible."
						<< " But it seems no value is specified for this property in the YAML file.";
			}
		}
	}
}

void Accessible::forwardProperty(const std::string& name, const Accessible& target, const std::string& targetValueName)
{
	Functors functors;
	auto found = target.m_functors.find(targetValueName);
	if (found != target.m_functors.end())
	{
		functors.getter = found->second.getter;
		functors.setter = found->second.setter;
		functors.encoder = found->second.encoder;
		functors.decoder = found->second.decoder;
		m_functors[name] = std::move(functors);
	}
	else
	{
		SURGSIM_FAILURE() << "Target does not have any setters or getters on property " << targetValueName;
	}
}

template<>
SurgSim::Math::Matrix44f convert(boost::any val)
{

	SurgSim::Math::Matrix44f floatResult;
	// Use try in case this conversion was created using a Matrix44f, in which case the any_cast will
	// still fail and throw an exception
	try
	{
		SurgSim::Math::Matrix44d result = boost::any_cast<SurgSim::Math::Matrix44d>(val);
		floatResult = result.cast<float>();
	}
	catch (boost::bad_any_cast&)
	{
		floatResult = boost::any_cast<SurgSim::Math::Matrix44f>(val);
	}
	return floatResult;
}

template<>
std::string convert(boost::any val)
{
	std::string result;
	try
	{
		result = std::string(boost::any_cast<const char*>(val));
	}
	catch (boost::bad_any_cast&)
	{
		result = std::string(boost::any_cast<std::string>(val));
	}
	return result;
}

}; // Framework
}; // SurgSim
