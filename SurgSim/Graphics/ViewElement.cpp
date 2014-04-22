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

#include "SurgSim/Graphics/ViewElement.h"

#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/View.h"

namespace SurgSim
{
namespace Graphics
{
ViewElement::ViewElement(const std::string& name, std::shared_ptr<View> view, std::shared_ptr<Camera> camera) :
	SceneElement(name),
	m_view(view),
	m_camera(camera)
{
	m_camera->setRenderGroupReference(SurgSim::Graphics::Representation::DefaultGroupName);
	setView(m_view);
	setCamera(m_camera);
}

ViewElement::~ViewElement()
{
}

bool ViewElement::setView(std::shared_ptr<View> view)
{
	m_view = view;
	m_view->setCamera(m_camera);
	return true;
}

std::shared_ptr<View> ViewElement::getView()
{
	return m_view;
}

void ViewElement::setCamera(std::shared_ptr<Camera> camera)
{
	m_camera = camera;
	if (m_view != nullptr)
	{
		m_view->setCamera(m_camera);
	}
}

std::shared_ptr<Camera> ViewElement::getCamera()
{
	return m_camera;
}

bool ViewElement::doInitialize()
{
	addComponent(m_view);
	addComponent(m_camera);
	return true;
}



}; // End of namespace Graphics
}; // End of namespace SurgSim std::shared_ptr<Camera> m_camera;