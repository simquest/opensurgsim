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

#ifndef SURGSIM_DEVICES_TRAKSTAR_TRAKSTARDEVICE_H
#define SURGSIM_DEVICES_TRAKSTAR_TRAKSTARDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Devices
{
class TrakstarScaffold;

SURGSIM_STATIC_REGISTRATION(TrakstarDevice);

/// A class implementing the communication with NDI's Trakstar electromagnetic tracker.
/// \par Application input provided by the device:
///   | type       | name              |                                        |
///   | ----       | ----              | ---                                    |
///   | pose       | "pose"            | %Device pose (units are meters).       |
///   | scalar     | "time"            | %The time returned with the record.    |
///   | scalar     | "quality"         | %The record's quality measure.         |
///
/// \par Application output used by the device: none.
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class TrakstarDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device.
	explicit TrakstarDevice(const std::string& uniqueName);

	SURGSIM_CLASSNAME(SurgSim::Devices::TrakstarDevice);

	/// Destructor.
	virtual ~TrakstarDevice();

	bool initialize() override;

	bool isInitialized() const override;

	/// \return The sensor ID.
	unsigned short getSensorId() const;

	/// Set the sensor ID, used by the SDK to select between attached sensors. Cannot be called after initialization.
	/// The default sensor ID is 0.
	/// \param id The sensor ID.
	void setSensorId(unsigned short id);

	/// \return The optional update rate.
	DataStructures::OptionalValue<double> getOptionalUpdateRate() const;

	/// Set the optional update rate. Cannot be called after initialization.
	/// \param rate The optional update rate.
	void setOptionalUpdateRate(DataStructures::OptionalValue<double> rate);

	/// Set the update rate.
	/// This call sets the update rate for the shared scaffold, so devices cannot set it to different values.
	/// The default (factory-set) update rate is 240Hz, but that can be changed 
	/// Cannot be called after initialization.
	/// \param rate The update rate in Hz, must be >=60Hz and <=765Hz.
	void setUpdateRate(double rate);

private:
	friend class TrakstarScaffold;

	bool finalize() override;

	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<TrakstarScaffold> m_scaffold;

	/// The sensor ID.
	unsigned short m_sensorId;

	/// The update rate requested for this sensor.
	DataStructures::OptionalValue<double> m_updateRate;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_TRAKSTAR_TRAKSTARDEVICE_H
