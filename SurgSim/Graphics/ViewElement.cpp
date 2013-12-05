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

#include "SurgSim/Devices/Keyboard/KeyboardDevice.h"
#include "SurgSim/Graphics/View.h"
#include "SurgSim/Graphics/ViewElement.h"

using SurgSim::Graphics::View;
using SurgSim::Graphics::ViewElement;

ViewElement::ViewElement(const std::string& name, std::shared_ptr<View> view) : SceneElement(name), m_view(view)
{
}

ViewElement::~ViewElement()
{
}

bool ViewElement::setView(std::shared_ptr<View> view)
{
	m_view = view;
	return true;
}

std::shared_ptr<View> ViewElement::getView() const
{
	return m_view;
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

std::shared_ptr<SurgSim::Device::KeyboardDevice> SurgSim::Graphics::ViewElement::getKeyboardDevice()
{
	static auto keyboardDevice = std::make_shared<SurgSim::Device::KeyboardDevice>("Keyboard");
	if (! keyboardDevice->isInitialized())
	{
		keyboardDevice->initialize();
	}
	return keyboardDevice;
}
