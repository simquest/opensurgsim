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

#include "SurgSim/Devices/Keyboard/KeyboardThread.h"

#include <SurgSim/Devices/Keyboard/keyboardScaffold.h>

namespace SurgSim
{
namespace Device
{

KeyboardThread::~KeyboardThread()
{
}

bool KeyboardThread::doInitialize()
{
	return true;
}

bool KeyboardThread::doStartUp()
{
	return true;
}

bool KeyboardThread::doUpdate(double dt)
{
	return m_scaffold->runInputFrame(m_deviceData);
}

void KeyboardThread::doBeforeStop()
{
	//m_scaffold->runAfterLastFrame(m_deviceData);
}

};  // namespace Device
};  // namespace SurgSim
