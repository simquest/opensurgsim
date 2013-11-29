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
#include "SurgSim/Framework/Assert.h"

using SurgSim::Framework::Timer;

TEST(TimerTest, Constructor)
{
	EXPECT_NO_THROW({std::shared_ptr<Timer> timer(new Timer());});
}

TEST(TimerTest, Starting)
{
	std::shared_ptr<Timer> timer = std::make_shared<Timer>();
	EXPECT_EQ(timer->getCurrentNumberOfFrames(), 0);
	EXPECT_EQ(timer->getNumberOfClockFails(), 0);
	EXPECT_THROW(timer->getCumulativeTime(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getAverageFramePeriod(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getAverageFrameRate(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getLastFramePeriod(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getLastFrameRate(), SurgSim::Framework::AssertionFailure);
}

TEST(TimerTest, SettingFrames)
{
	std::shared_ptr<Timer> timer(new Timer());
	timer->endFrame();
	EXPECT_EQ(timer->getCurrentNumberOfFrames(), 1);
	EXPECT_EQ(timer->getAverageFrameRate(), timer->getLastFrameRate());
	EXPECT_EQ(timer->getAverageFramePeriod(), timer->getLastFramePeriod());
	EXPECT_EQ(timer->getLastFramePeriod(), timer->getCumulativeTime());

	timer->setMaxNumberOfFrames(3);
	timer->start();
	for (auto i = 0; i < 5; ++i)
	{
		timer->markFrame();
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
		timer1->markFrame();
	}
	// timer1's frames include timer2's frames and the for-loop operations, so timer1's frame period should be longer
	// than timer2's frames (or at least no shorter).
	EXPECT_GE(timer1->getAverageFramePeriod(), timer2->getAverageFramePeriod());
	EXPECT_GT(timer1->getCumulativeTime(), timer1->getLastFramePeriod());
}


