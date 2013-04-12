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

#include "Manager.h"

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/Scene.h>
#include <SurgSim/Graphics/ViewComponent.h>

using SurgSim::Graphics::Manager;
using SurgSim::Graphics::Representation;
using SurgSim::Graphics::Scene;
using SurgSim::Graphics::ViewComponent;


Manager::Manager(std::shared_ptr<Scene> scene) :
	BasicThread("Graphics Manager"),
	m_scene(scene)
{
}

Manager::~Manager()
{

}

bool Manager::addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = true;
	std::shared_ptr<Graphics::Representation> representation = 
		std::dynamic_pointer_cast<Graphics::Representation>(component);
	if (representation != nullptr)
	{
		result = m_scene->addActor(representation->getActor());
	}

	std::shared_ptr<Graphics::ViewComponent> view = 
		std::dynamic_pointer_cast<Graphics::ViewComponent>(component);
	if (view != nullptr)
	{
		m_views.insert(view);
		result = true;
	}
	return result;
}

bool Manager::removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
{
	bool result = false;
	std::shared_ptr<Graphics::Representation> representation = 
		std::dynamic_pointer_cast<Graphics::Representation> (component);
	if (representation != nullptr)
	{
		result = m_scene->removeActor(representation->getActor());
	}

	std::shared_ptr<Graphics::ViewComponent> view = 
		std::dynamic_pointer_cast<Graphics::ViewComponent>(component);
	if (m_views.find(view) != m_views.end())
	{
		m_views.erase(view);
		result = true;
	}
	return result;
}

bool Manager::doInitialize()
{
	return true;
}

bool Manager::doStartUp()
{
	return true;
}

bool Manager::doUpdate(double dt)
{
	if (m_scene != nullptr)
	{
		m_scene->update(dt);
	}
	for (auto it = m_views.begin(); it != m_views.end(); ++it)
	{
		(*it)->update(dt);
	}
	return true;
}