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

#include "SurgSim/Graphics/OsgLog.h"
#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"

TEST(OsgLogTests, MessageTest)
{
	osg::NotifyHandler* pOsgLog = new MockOsgLog;
	osg::setNotifyHandler(pOsgLog);

	EXPECT_EQ("", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::ALWAYS) << "osg::ALWAYS Test" << std::endl;
	EXPECT_EQ("CRITICAL osg::ALWAYS Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::FATAL) << "osg::FATAL Test" << std::endl;
	EXPECT_EQ("CRITICAL osg::FATAL Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::WARN) << "osg::WARN Test" << std::endl;
	EXPECT_EQ("WARNING osg::WARN Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::NOTICE) << "osg::NOTICE Test" << std::endl;
	EXPECT_EQ("INFO osg::NOTICE Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::INFO) << "osg::INFO Test" << std::endl;
	EXPECT_EQ("INFO osg::INFO Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::DEBUG_INFO) << "osg::DEBUG_INFO Test" << std::endl;
	EXPECT_EQ("DEBUG osg::DEBUG_INFO Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::DEBUG_FP) << "osg::DEBUG_FP Test" << std::endl;
	EXPECT_EQ("DEBUG osg::DEBUG_FP Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());

	osg::notify(osg::DEBUG_FP) << "osg::DEBUG_FP Test" << std::endl;
	EXPECT_EQ("DEBUG osg::DEBUG_FP Test\n", dynamic_cast<MockOsgLog*>(pOsgLog)->getMessage());
}
