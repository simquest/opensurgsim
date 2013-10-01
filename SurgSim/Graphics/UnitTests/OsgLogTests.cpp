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

#include <gtest/gtest.h>
#include <iostream>

#include <SurgSim/Graphics/OsgLog.h>
#include <SurgSim/Graphics/UnitTests/MockOsgObjects.h>

TEST(OsgLogTests, MessageTest)
{
	osg::NotifyHandler* pOsgLog = new MockOsgLog;
	osg::setNotifyHandler(pOsgLog);

	// Level is below default level (osg::INFO)
	osg::notify(osg::DEBUG_FP) << "Test" << std::endl;
	EXPECT_EQ("", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	// Level is above default level (osg::INFO)
	osg::notify(osg::FATAL) << "Fatal" << std::endl;
	EXPECT_EQ("Fatal\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());
}
