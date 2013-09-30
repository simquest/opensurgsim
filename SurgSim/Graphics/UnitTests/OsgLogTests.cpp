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

/// \file
/// Tests for the SURGSIM_LOG_*() macros and the related classes.

#include <gtest/gtest.h>
#include <iostream>
#include <SurgSim/Graphics/OsgLog.h>

TEST(LoggerTest, MessageTest)
{
	SurgSim::Graphics::OsgLog osgLog;
	osg::NotifyHandler* pOsgLog = &osgLog;

	/*osg::setNotifyHandler(pOsgLog);*/
	osg::notify()<<"Test info\n";

int x;
std::cin>>x;
std::cout<<x<<std::endl;

std::cin>>x;
}
