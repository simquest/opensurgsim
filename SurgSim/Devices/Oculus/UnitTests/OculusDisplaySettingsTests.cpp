// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include <gtest/gtest.h>
#include <osg/DisplaySettings>

#include "SurgSim/Devices/Oculus/OculusDisplaySettings.h"
#include "SurgSim/Math/Matrix.h"

namespace
{
const double epsilon = 1e-10;
}

using SurgSim::Devices::OculusDisplaySettings;

TEST(OculusDisplaySettingsTest, Constructor)
{
	EXPECT_NO_THROW(OculusDisplaySettings displaySettings);
	osg::ref_ptr<osg::DisplaySettings> osgDisplaySettings = new osg::DisplaySettings;
	EXPECT_NO_THROW(OculusDisplaySettings displaySettings(osgDisplaySettings));
}

TEST(OculusDisplaySettingsTest, SetAndGetProjectionMatrix)
{
	OculusDisplaySettings displaySettings;
	SurgSim::Math::Matrix44d leftProjectionMatrix = SurgSim::Math::Matrix44d::Identity() * 2.0;
	SurgSim::Math::Matrix44d rightProjectionMatrix = SurgSim::Math::Matrix44d::Identity() * 3.0;

	EXPECT_NO_THROW(displaySettings.setLeftEyeProjectionMatrix(leftProjectionMatrix));
	EXPECT_NO_THROW(displaySettings.setRightEyeProjectionMatrix(rightProjectionMatrix));

	SurgSim::Math::Matrix44d leftRetrieved;
	EXPECT_NO_THROW(leftRetrieved = displaySettings.getLeftEyeProjectionMatrix());
	EXPECT_TRUE(leftProjectionMatrix.isApprox(leftRetrieved, epsilon));

	SurgSim::Math::Matrix44d rightRetrieved;
	EXPECT_NO_THROW(rightRetrieved = displaySettings.getRightEyeProjectionMatrix());
	EXPECT_TRUE(rightProjectionMatrix.isApprox(rightRetrieved, epsilon));
}
