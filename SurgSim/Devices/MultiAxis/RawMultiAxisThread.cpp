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

#include "SurgSim/Devices/MultiAxis/RawMultiAxisThread.h"

namespace SurgSim
{
namespace Devices
{

RawMultiAxisThread::RawMultiAxisThread(RawMultiAxisScaffold* scaffold, RawMultiAxisScaffold::DeviceData* deviceData) :
	BasicThread("RawMultiAxis thread"),
	m_scaffold(scaffold),
	m_deviceData(deviceData)
{
	setRate(100.0);
}

RawMultiAxisThread::~RawMultiAxisThread()
{
}

bool RawMultiAxisThread::doInitialize()
{
	return true;
}

bool RawMultiAxisThread::doStartUp()
{
	return true;
}

bool RawMultiAxisThread::doUpdate(double dt)
{
	return m_scaffold->runInputFrame(m_deviceData);
}

void RawMultiAxisThread::doBeforeStop()
{
	m_scaffold->runAfterLastFrame(m_deviceData);
}

};  // namespace Devices
};  // namespace SurgSim
