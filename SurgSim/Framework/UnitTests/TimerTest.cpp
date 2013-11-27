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
#include "SurgSim/Framework/Timer.h"
//#include "MockObjects.h"  //NOLINT

using SurgSim::Framework::Timer;

TEST(TimerTest, Constructor)
{
	EXPECT_NO_THROW({std::shared_ptr<Timer> timer(new Timer());});

}

TEST(TimerTest, Starting)
{
	std::shared_ptr<Timer> timer(new Timer());
	EXPECT_EQ(timer->getCurrentNumberOfFrames(), 0);
	EXPECT_EQ(timer->getNumberOfClockFails(), 0);
}

TEST(TimerTest, SettingFrames)
{
	std::shared_ptr<Timer> timer(new Timer());
	timer->endFrame();
	EXPECT_EQ(timer->getCurrentNumberOfFrames(), 1);
	EXPECT_EQ(timer->getAverageFrameRate(), timer->getLastFrameRate());
	EXPECT_EQ(timer->getAverageFramePeriod(), timer->getLastFramePeriod());

	timer->start();
	timer->setNumberOfFrames(3);
	for (auto i = 0; i < 5; ++i)
	{
		timer->endFrame();
	}
	EXPECT_EQ(timer->getCurrentNumberOfFrames(), 3);
}

TEST(TimerTest, Comparison)
{
	std::shared_ptr<Timer> timer1(new Timer());
	std::shared_ptr<Timer> timer2(new Timer());

	for (auto i = 0; i < 100; ++i)
	{
		timer2->beginFrame();
		timer2->endFrame();
		timer1->endFrame();
	}
	EXPECT_TRUE(timer1->getAverageFramePeriod() >= timer2->getAverageFramePeriod());
}


