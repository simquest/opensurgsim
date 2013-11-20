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

#ifndef SURGSIM_DEVICES_KEYBOARD_KEYBOARDHANDLER_H
#define SURGSIM_DEVICES_KEYBOARD_KEYBOARDHANDLER_H

#include <SurgSim/Devices/Keyboard/KeyboardScaffold.h>

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

class KeyboardHandler : public osgGA::GUIEventHandler
{
public:

	KeyboardHandler() : m_keyboardScaffold(KeyboardScaffold::getOrCreateSharedInstance())
	{
	}

	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
	{
		switch(ea.getEventType())
		{
		case(osgGA::GUIEventAdapter::KEYDOWN) :
		{// Note that we are setting the modifier mask here instead of the modifier itself
			m_keyboardScaffold.lock()->updateDevice(ea.getUnmodifiedKey(), ea.getModKeyMask());
			std::cerr << "Key code for pressed key: "<< ea.getKey() << std::endl;
			return true;
		}
		case(osgGA::GUIEventAdapter::KEYUP) :
			{
				m_keyboardScaffold.lock()->updateDevice(-1, 0);
				return true;
			}
		default:
			return false;
		}
	}

private:
	std::weak_ptr<KeyboardScaffold> m_keyboardScaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_KEYBOARD_KEYBOARDHANDLER_H