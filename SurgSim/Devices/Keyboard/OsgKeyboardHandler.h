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

#ifndef SURGSIM_DEVICES_KEYBOARD_OSGKEYBOARDHANDLER_H
#define SURGSIM_DEVICES_KEYBOARD_OSGKEYBOARDHANDLER_H

#include <memory>

#include <osgGA/GUIEventHandler>

namespace SurgSim
{
namespace Device
{

class KeyboardScaffold;

class OsgKeyboardHandler : public osgGA::GUIEventHandler
{
public:
	/// Constructor
	OsgKeyboardHandler();

	/// Method to handle GUI event
	/// \param ea A osgGA::GUIEventAdapter
	/// \param aa A osgGA::GUIActionAdapter (required by this virtual method)
	/// \return True if the event has been handled by this method; Otherwise, false.
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

private:
	/// A back pointer to the scaffold which owns this handle
	std::weak_ptr<KeyboardScaffold> m_keyboardScaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_KEYBOARD_OSGKEYBOARDHANDLER_H