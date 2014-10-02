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

#include "SurgSim/Devices/Nimble/NimbleThread.h"

#include "SurgSim/Devices/Nimble/NimbleScaffold.h"

namespace SurgSim
{
namespace Device
{

NimbleThread::NimbleThread(NimbleScaffold* scaffold)
	: BasicThread("Nimble thread"), m_scaffold(scaffold)
{
	setRate(1000.0);
}

NimbleThread::~NimbleThread()
{
}

bool NimbleThread::doUpdate(double dt)
{
	return m_scaffold->update();
}

bool NimbleThread::doInitialize()
{
	return m_scaffold->initialize();
}

bool NimbleThread::doStartUp()
{
	return true;
}

void NimbleThread::doBeforeStop()
{
	return m_scaffold->finalize();
}

};  // namespace Device
};  // namespace SurgSim
