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
	/// TrackIR sample rate: 120FPS.
	/// Default update rate is set by BasicThread constructor to 30Hz
	/// \param scaffold The TrackIRScaffold updated by this thread
	/// \param deviceData Corresponds to the TrackIRScaffold::DeviceData updated by this thread
	TrackIRThread(TrackIRScaffold* scaffold, TrackIRScaffold::DeviceData* deviceData);

	/// Destructor
	virtual ~TrackIRThread();

protected:
	/// Initialize this thread.
	/// \return True on success, false otherwise.
	bool doInitialize() override;
	/// Start up this thread.
	/// \return True on success, false otherwise.
	bool doStartUp() override;
	/// Update work of this thread.
	/// \param dt The time step.
	/// \return True on success, false otherwise.
	bool doUpdate(double dt) override;

private:
	// Pointer to the scaffold which will be updated by this thread.
	TrackIRScaffold* m_scaffold;
	// Pointer to the DeviceData object which will be updated by the scaffold.
	TrackIRScaffold::DeviceData* m_deviceData;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRACKIR_TRACKIRTHREAD_H
