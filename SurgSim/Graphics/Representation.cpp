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

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Framework/Scene.h"


#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Framework/Log.h"

#include <boost/algorithm/string.hpp>

using SurgSim::Graphics::Material;

namespace SurgSim
{
namespace Graphics
{
const std::string Representation::DefaultGroupName = "__OssDefault__";
const std::string Representation::DefaultHudGroupName = "__OssDefaulHud__";

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, std::vector<std::string>, GroupReferences,
									  getGroupReferences, setGroupReferences);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, bool, DrawAsWireFrame,
									  getDrawAsWireFrame, setDrawAsWireFrame);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, bool, GenerateTangents,
									  isGeneratingTangents, setGenerateTangents);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, std::string, MaterialReference,
									  getMaterialReference, setMaterialReference);

	addGroupReference(DefaultGroupName);
}

bool Representation::addGroupReference(const std::string& name)
{
	bool result = false;
	if (!checkAwake("addGroupReference"))
	{
		auto insertion = m_groups.insert(name);
		result = insertion.second;
	}
	return result;
}

void Representation::addGroupReferences(const std::vector<std::string>& groups)
{
	if (!checkAwake("addGroupReferences"))
	{
		for (auto it = groups.cbegin(); it != groups.cend(); ++it)
		{
			addGroupReference(*it);
		}
	}
}

void Representation::setGroupReferences(const std::vector<std::string>& groups)
{
	if (!checkAwake("setGroupReferences"))
	{
		m_groups.clear();
		for (auto it = groups.cbegin(); it != groups.cend(); ++it)
		{
			addGroupReference(*it);
		}
	}
}

std::vector<std::string> Representation::getGroupReferences() const
{
	return std::vector<std::string>(std::begin(m_groups), std::end(m_groups));
}

void Representation::clearGroupReferences()
{
	if (!checkAwake("clearGroupReference"))
	{
		m_groups.clear();
	}
}

bool Representation::doWakeUp()
{
	if (getMaterial() == nullptr && !m_materialReference.empty())
	{
		std::vector<std::string> names;
		boost::split(names, m_materialReference, boost::is_any_of("/"));

		SURGSIM_ASSERT(names.size() == 2)
				<< "Material reference needs to have 2 parts <scenelement>/<component>, '" << m_materialReference
				<< "' in " << getFullName() << " doesn't.";

		auto material = getScene()->getComponent(names[0], names[1]);
		if (material != nullptr)
		{
			setMaterial(material);
		}
		else
		{
			SURGSIM_LOG_WARNING(Framework::Logger::getLogger("Graphics/Representation"))
					<< "Can't find material " << m_materialReference << " in Scene, rendering of " << getFullName()
					<< " is going to be compromised.";
		}
	}
	return true;
}

bool Representation::removeGroupReference(const std::string& name)
{
	bool result = false;
	if (!checkAwake("removeGroupReference"))
	{
		result = (m_groups.erase(name) != 0u);
	}
	return result;
}

void Representation::setGroupReference(const std::string& group)
{
	if (!checkAwake("setGroupReference"))
	{
		clearGroupReferences();
		m_groups.insert(group);
	}
}

bool Representation::checkAwake(const std::string& functionName)
{
	if (isAwake())
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
				"Representation::" << functionName << "() was called while the component " <<
				"was already awake for component " << getName() << " this has no effect and should be avoided.";
	}
	return isAwake();
}

Representation::~Representation()
{

}

void Representation::setMaterialReference(const std::string& materialReference)
{
	m_materialReference = materialReference;
}

std::string Representation::getMaterialReference() const
{
	return m_materialReference;
}

}; // namespace Graphics
}; // namespace SurgSim

