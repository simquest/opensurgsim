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

#ifndef SURGSIM_DEVICES_KEYBOARD_KEYBOARDTHREAD_H
#define SURGSIM_DEVICES_KEYBOARD_KEYBOARDTHREAD_H

#include <memory>
#include <string>

#include <SurgSim/Framework/BasicThread.h>
#include <SurgSim/Devices/Keyboard/KeyboardScaffold.h>


namespace SurgSim
{
namespace Device
{

/// A class implementing the thread context for sampling RawMultiAxis devices.
/// \sa SurgSim::Device::KeyboardScaffold
class KeyboardThread : public SurgSim::Framework::BasicThread
{
public:
	explicit KeyboardThread(KeyboardScaffold* scaffold, KeyboardScaffold::DeviceData* deviceData) :
		BasicThread("Keyboard thread"),
		m_scaffold(scaffold),
		m_deviceData(deviceData)
	{
		setRate(100.0);
	}

	virtual ~KeyboardThread();

protected:
	virtual bool doInitialize() override;
	virtual bool doStartUp() override;
	virtual bool doUpdate(double dt) override;
	virtual void doBeforeStop() override;

private:
	KeyboardScaffold* m_scaffold;
	KeyboardScaffold::DeviceData* m_deviceData;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_KEYBOARD_KEYBOARDTHREAD_H
