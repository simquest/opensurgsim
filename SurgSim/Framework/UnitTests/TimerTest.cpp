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
	EXPECT_NO_THROW(timer->getCumulativeTime());
	EXPECT_EQ(0.0, timer->getCumulativeTime());
	EXPECT_THROW(timer->getAverageFramePeriod(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getAverageFrameRate(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getLastFramePeriod(), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(timer->getLastFrameRate(), SurgSim::Framework::AssertionFailure);
}

TEST(TimerTest, SettingFrames)
{
	std::shared_ptr<Timer> timer = std::make_shared<Timer>();
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
	timer->setMaxNumberOfFrames(2);
	EXPECT_EQ(timer->getCurrentNumberOfFrames(), 2);
}

TEST(TimerTest, Comparison)
{
	std::shared_ptr<Timer> timer1 = std::make_shared<Timer>();
	std::shared_ptr<Timer> timer2 = std::make_shared<Timer>();

	for (auto i = 0; i < 100; ++i)
	{
		timer2->beginFrame();
		timer2->endFrame();
		timer1->markFrame();
	}
	// timer1's frames include timer2's frames and the for-loop operations, so timer1's frame period should be longer
	// than timer2's frames (or at least no shorter).
	if ((timer1->getNumberOfClockFails() == 0) && (timer2->getNumberOfClockFails() == 0))
	{
		EXPECT_GE(timer1->getAverageFramePeriod(), timer2->getAverageFramePeriod());
		EXPECT_GT(timer1->getCumulativeTime(), timer1->getLastFramePeriod());

		EXPECT_GE(timer1->getMaxFramePeriod(), timer2->getMaxFramePeriod());
		EXPECT_GE(timer1->getMinFramePeriod(), timer2->getMinFramePeriod());
	}
}

TEST(TimerTest, GetWithoutAnyFrames)
{
	std::shared_ptr<Timer> timer = std::make_shared<Timer>();

	EXPECT_NO_THROW(timer->getCumulativeTime());
	EXPECT_ANY_THROW(timer->getAverageFramePeriod());
	EXPECT_ANY_THROW(timer->getAverageFrameRate());
	EXPECT_ANY_THROW(timer->getLastFramePeriod());
	EXPECT_ANY_THROW(timer->getLastFrameRate());
	EXPECT_ANY_THROW(timer->getMaxFramePeriod());
	EXPECT_ANY_THROW(timer->getMinFramePeriod());
}
