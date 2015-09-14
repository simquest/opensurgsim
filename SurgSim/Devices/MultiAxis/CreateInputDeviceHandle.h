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

#ifndef SURGSIM_DEVICES_MULTIAXIS_CREATEINPUTDEVICEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_CREATEINPUTDEVICEHANDLE_H

#include <string>
#include <memory>
#include <vector>

namespace SurgSim
{
namespace Framework
{
class Logger;
};  // namespace Framework

namespace Devices
{
class SystemInputDeviceHandle;


/// Opens the given path and creates an access wrapper for the device.
/// \param path	Full pathname for the device.
/// \param logger	The logger to be used by the device.
/// \return	The created device object, or an empty unique_ptr on failure.
std::unique_ptr<SystemInputDeviceHandle> createInputDeviceHandle(const std::string& path,
		std::shared_ptr<SurgSim::Framework::Logger> logger);

/// Enumerates input devices.
/// \param logger	The logger to be used during enumeration.
/// \return	A list of device paths.
std::vector<std::string> enumerateInputDevicePaths(SurgSim::Framework::Logger* logger);


};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_CREATEINPUTDEVICEHANDLE_H
