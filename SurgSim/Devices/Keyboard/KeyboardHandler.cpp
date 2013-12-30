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

#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Devices/Keyboard/KeyboardHandler.h"
#include "SurgSim/Devices/Keyboard/KeyboardScaffold.h"

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

KeyboardHandler::KeyboardHandler() : m_keyboardScaffold(KeyboardScaffold::getOrCreateSharedInstance())
{
}

bool KeyboardHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	switch(ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::KEYDOWN) :
	{// Note that we are setting the modifier mask here instead of the modifier itself
		m_keyboardScaffold.lock()->updateDevice(ea.getUnmodifiedKey(), ea.getModKeyMask());
		return true;
	}
	case(osgGA::GUIEventAdapter::KEYUP) :
	{
		m_keyboardScaffold.lock()->updateDevice(KeyCode::NONE, ModKeyMask::MODKEY_NONE);
		return true;
	}
	default:
		return false;
	}
}

};  // namespace Device
};  // namespace SurgSim
