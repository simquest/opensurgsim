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

#ifndef SURGSIM_DEVICES_LABJACK_LINUX_LABJACKCONSTANTS_H
#define SURGSIM_DEVICES_LABJACK_LINUX_LABJACKCONSTANTS_H

#include <array>

namespace SurgSim
{
namespace Devices
{
namespace LabJack
{

/// The maximum size of a read or write buffer for labjackusb driver.
static const int MAXIMUM_BUFFER = 64;

};  // namespace LabJack
};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LINUX_LABJACKCONSTANTS_H
