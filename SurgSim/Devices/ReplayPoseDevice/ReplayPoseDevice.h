// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DEVICES_REPLAYPOSEDEVICE_REPLAYPOSEDEVICE_H
#define SURGSIM_DEVICES_REPLAYPOSEDEVICE_REPLAYPOSEDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{

class ReplayPoseScaffold;

SURGSIM_STATIC_REGISTRATION(ReplayPoseDevice);

/// A class implementing the replay pose device, which is a pretend device that replays a recorded motion from a file
///
/// This can be useful not only for writing tests, but also as a way to replace real hardware devices in situations
/// where the simulator needs to be run but the hardware is not currently available.
///
/// \par Application input provided by the device:
///   | type       | name                  |                                                      |
///   | ----       | ----                  | ---                                                  |
///   | pose       | "pose"                | %Device pose (units are meters).                     |
///
/// \sa SurgSim::Input::CommonDevice
class ReplayPoseDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit ReplayPoseDevice(const std::string& uniqueName);

	/// \return The filename from which the input poses are read (default is 'ReplayPoseDevice.txt')
	const std::string getFileName() const;

	/// \param fileName from which the input poses will be read (default is 'ReplayPoseDevice.txt')
	void setFileName(const std::string& fileName);

	/// \return The rate (in Hz) at which the device is running (1KHz is the default)
	double getRate() const;

	/// \param rate The rate (in Hz) at which the device will run (1KHz is the default)
	/// \exception SurgSim::Framework::AssertionFailure if the method is called after initialization
	void setRate(double rate);

	SURGSIM_CLASSNAME(SurgSim::Devices::ReplayPoseDevice);

	virtual ~ReplayPoseDevice();

	bool initialize() override;

	bool isInitialized() const override;

private:
	friend class ReplayPoseScaffold;

	bool finalize() override;

	std::shared_ptr<ReplayPoseScaffold> m_scaffold;

	/// The filename to read the input transform from
	std::string m_fileName;

	/// The rate to run the device at (i.e. at which rate the information is populated).
	/// This is independent of the record being replayed, the motion will be real-time (interpolation may occur).
	double m_rate;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_REPLAYPOSEDEVICE_REPLAYPOSEDEVICE_H
