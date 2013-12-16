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

#ifndef SURGSIM_DEVICES_TRACKIR_TRACKIRTHREAD_H
#define SURGSIM_DEVICES_TRACKIR_TRACKIRTHREAD_H

#include <memory>
#include <string>

#include "SurgSim/Framework/BasicThread.h"
#include "SurgSim/Devices/TrackIR/TrackIRScaffold.h"


namespace SurgSim
{
namespace Device
{

/// A class implementing the thread context for sampling TrackIR devices.
/// \sa SurgSim::Device::TrackIRScaffold
class TrackIRThread : public SurgSim::Framework::BasicThread
{
public:
	/// Constructor
	/// \param scaffold Corresponds to the TrackIRScaffold updated by this thread
	/// \param deviceData Corresponds to the TrackIRScaffold::DeviceData updated by this thread
	TrackIRThread(TrackIRScaffold* scaffold, TrackIRScaffold::DeviceData* deviceData);

	/// Destructor
	virtual ~TrackIRThread();

protected:
	///@{
	/// Overridden from SurgSim::Framework::BasicThread
	virtual bool doInitialize() override;
	virtual bool doStartUp() override;
	virtual bool doUpdate(double dt) override;
	///@}

private:
	TrackIRScaffold* m_scaffold;
	TrackIRScaffold::DeviceData* m_deviceData;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRACKIR_TRACKIRTHREAD_H
