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

#ifndef SURGSIM_DEVICES_TRACKIR_TRACKIRDEVICE_H
#define SURGSIM_DEVICES_TRACKIR_TRACKIRDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class TrackIRScaffold;

class TrackIRDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	TrackIRDevice(const std::string& uniqueName);

	/// Destructor.
	virtual ~TrackIRDevice();

	///@{
	/// Overridden from SurgSim::Input::CommonDevice	
	virtual bool initialize() override;
	virtual bool finalize() override;
	///@}

	/// Check whether this device is initialized.
	bool isInitialized() const;

//private:
	friend class TrackIRScaffold;
	
	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<TrackIRScaffold> m_scaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRACKIR_TRACKIRDEVICE_H
