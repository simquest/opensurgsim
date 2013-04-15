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

#include "ViewElement.h"

#include <SurgSim/Graphics/CameraRepresentation.h>
#include <SurgSim/Graphics/ViewComponent.h>

using SurgSim::Graphics::CameraRepresentation;
using SurgSim::Graphics::ViewElement;
using SurgSim::Graphics::ViewComponent;

ViewElement::ViewElement(const std::string& name, std::shared_ptr<SurgSim::Graphics::ViewComponent> component) : 
SceneElement(name), m_view(component)
{
	m_camera = std::make_shared<CameraRepresentation>(m_view->getCamera());
}
ViewElement::~ViewElement()
{

}

void ViewElement::setScreen(unsigned int screen)
{
	m_view->setScreen(screen);
}
void ViewElement::setFullScreen(bool fullscreen)
{
	m_view->setFullScreen(fullscreen);
}
void ViewElement::setWindow(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	m_view->setWindow(x, y, width, height);
}

bool ViewElement::doInitialize()
{
	addComponent(m_view);

	return true;
}
bool ViewElement::doWakeUp()
{
	return true;
}