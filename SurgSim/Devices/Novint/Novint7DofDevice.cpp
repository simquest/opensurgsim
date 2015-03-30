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

#include "SurgSim/Devices/Novint/Novint7DofDevice.h"

namespace SurgSim
{
namespace Device
{

Novint7DofDevice::Novint7DofDevice(const std::string& uniqueName, const std::string& initializationName) :
	NovintCommonDevice(uniqueName, initializationName)
{
}

Novint7DofDevice::~Novint7DofDevice()
{
}

bool Novint7DofDevice::is7DofDevice() const
{
	return true;
}

};  // namespace Device
};  // namespace SurgSim
