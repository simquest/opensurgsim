// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Tests for the OculusDisplaySettings class.

#include <gtest/gtest.h>

#include "SurgSim/Devices/Oculus/OculusDisplaySettings.h"
#include "SurgSim/Devices/Oculus/OculusDevice.h"

using SurgSim::Device::OculusDevice;
using SurgSim::Device::OculusDisplaySettings;

TEST(OculusDisplaySettings, Init)
{
	EXPECT_NO_THROW(OculusDisplaySettings oculusDisplaySettings;);
}

TEST(OculusDisplaySettings, RetrieveProjectionMatrix)
{
	OculusDisplaySettings oculusDisplaySettings;
	EXPECT_THROW(oculusDisplaySettings.retrieveDeviceProjectionMatrix("Non-exist_Device"), SurgSim::Framework::AssertionFailure);

	OculusDevice oculusDevice("oculus");
	oculusDevice.initialize();
	EXPECT_NO_THROW(oculusDisplaySettings.retrieveDeviceProjectionMatrix("oculus"));
}
