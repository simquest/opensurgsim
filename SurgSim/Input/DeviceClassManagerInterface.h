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

#ifndef SURGSIM_INPUT_DEVICE_CLASS_MANAGER_INTERFACE_H
#define SURGSIM_INPUT_DEVICE_CLASS_MANAGER_INTERFACE_H

#include <memory>
#include <string>

#include <SurgSim/Input/DeviceInterface.h>

namespace SurgSim
{
namespace Input
{

/// Interface for an object that creates and manages and particular class of devices (multi-axis human interface
/// devices; Novint Technologies haptic devices; etc.).
///
// TODO(bert) This API really ought to include a method that creates a device, but right now we don't have any
// uniform signature for that (each device type uses a different argument list).  Future configurability work
// should try to resolve that...
class DeviceClassManagerInterface
{
public:
	/// Virtual destructor (empty).
	virtual ~DeviceClassManagerInterface() {};
};

};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_DEVICE_CLASS_MANAGER_INTERFACE_H
