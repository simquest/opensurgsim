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

#ifndef SURGSIM_DEVICES_LABJACK_LABJACKTHREAD_H
#define SURGSIM_DEVICES_LABJACK_LABJACKTHREAD_H

#include <memory>

#include "SurgSim/Framework/BasicThread.h"
#include "SurgSim/Devices/LabJack/LabJackScaffold.h"

namespace SurgSim
{
namespace Device
{

/// A class implementing the thread context for communicating with LabJack devices.
/// \sa SurgSim::Device::LabJackScaffold
class LabJackThread : public SurgSim::Framework::BasicThread
{
public:
	explicit LabJackThread(LabJackScaffold* scaffold, LabJackScaffold::DeviceData* deviceData);

	virtual ~LabJackThread();

protected:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

private:
	LabJackScaffold* m_scaffold;
	LabJackScaffold::DeviceData* m_deviceData;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LABJACKTHREAD_H
