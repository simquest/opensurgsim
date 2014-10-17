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

#ifndef SURGSIM_DEVICES_OPENNI_OPENNIDEVICE_H
#define SURGSIM_DEVICES_OPENNI_OPENNIDEVICE_H

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class OpenNIScaffold;

/// A class implementing the communication with one OpenNI compatible depth camera
///
/// \par Application input provided by the device:
///   | type       | name              |                                                                            |
///   | ----       | ----              | ---                                                                        |
///   | image      | "color"           | Color image (RGB) of floats, each pixel value is between 0 and 1.          |
///   | image      | "depth"           | Depth image of floats, each pixel value is depth from the camera in meters.|
///
/// \par Application output used by the device: none.
///
/// \sa SurgSim::Device::OpenNIScaffold
class OpenNIDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param name A unique name for the device that will be used by the application.
	explicit OpenNIDevice(const std::string& name);

	/// Destructor.
	virtual ~OpenNIDevice();

	virtual bool initialize() override;

	virtual bool finalize() override;

	/// Check wheter this device is initialized.
	bool isInitialized() const;

private:
	friend class OpenNIScaffold;

	std::shared_ptr<OpenNIScaffold> m_scaffold;
};

};
};

#endif //SURGSIM_DEVICES_OPENNI_OPENNIDEVICE_H
