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

#include "SurgSim/Devices/MultiAxis/SystemInputDeviceHandle.h"

#if defined(SURGSIM_MULTIAXIS_USING_LINUX_INPUT)
#include <SurgSim/Devices/MultiAxis/LinuxInputDeviceHandle.h>
#elif defined(SURGSIM_MULTIAXIS_USING_WINDOWS_HID)
#include <SurgSim/Devices/MultiAxis/Win32HidDeviceHandle.h>
#endif


namespace SurgSim
{
namespace Device
{

SystemInputDeviceHandle::SystemInputDeviceHandle()
{
}

SystemInputDeviceHandle::~SystemInputDeviceHandle()
{
}

std::unique_ptr<SystemInputDeviceHandle> SystemInputDeviceHandle::open(
	const std::string& path, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
#if defined(SURGSIM_MULTIAXIS_USING_LINUX_INPUT)
	return LinuxInputDeviceHandle::open(path, logger);
#elif defined(SURGSIM_MULTIAXIS_USING_WINDOWS_HID)
	return Win32HidDeviceHandle::open(path, logger);
#else
#error No known derived SystemInputDeviceHandle device class is applicable!
error: No known derived SystemInputDeviceHandle device class is applicable!
#endif
}

};  // namespace Device
};  // namespace SurgSim
