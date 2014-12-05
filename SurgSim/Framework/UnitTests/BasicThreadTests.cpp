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

// Original barrier.hpp
// Copyright (C) 2002-2003
// David Moore, William E. Kempf
// Copyright (C) 2007-8 Anthony Williams
//
//  Distributed under the Boost Software License, Version 1.0. (See accompanying
//  file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#include <gtest/gtest.h>
#include <boost/thread.hpp>


#include "SurgSim/Framework/BasicThread.h"
#include "MockObjects.h"  //NOLINT

namespace SurgSim
{
namespace Framework
{

TEST(BasicThreadTest, Instantiation)
{
	MockThread m;
	EXPECT_FALSE(m.isInitialized());
	EXPECT_FALSE(m.isRunning());

}

TEST(BasicThreadTest, Running)
{
	MockThread m(10);
	m.start(nullptr);

	m.getThread().join();

	EXPECT_EQ(0, m.count);
}

TEST(BasicThreadTest, Stop)
{
	MockThread m;
	m.start(nullptr);

	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	EXPECT_TRUE(m.isRunning());
	m.stop();

	EXPECT_TRUE(m.didBeforeStop);
	EXPECT_FALSE(m.isRunning());
}

TEST(BasicThreadTest, StopWithoutSleep)
{
	for (int i = 0;  i < 10;  ++i)
	{
		MockThread m;
		m.count = 1000000;
		m.start(nullptr);

		// Stopping right away should not create a race condition.
		m.stop();

		EXPECT_TRUE(m.didBeforeStop);
		EXPECT_FALSE(m.isRunning());
	}
}

TEST(BasicThreadTest, RunTimeManagement)
{
	MockThread m;
	EXPECT_EQ(-1, m.count);
	std::shared_ptr<Barrier> barrier = std::make_shared<Barrier>(2);
	EXPECT_FALSE(m.didInitialize);
	EXPECT_FALSE(m.didStartUp);
	m.start(barrier);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_TRUE(m.didInitialize);
	EXPECT_FALSE(m.didStartUp);
	barrier->wait(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_TRUE(m.didInitialize);
	EXPECT_TRUE(m.didStartUp);
	EXPECT_FALSE(m.isRunning());
	barrier->wait(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_TRUE(m.isRunning());

	m.stop();
}

TEST(BasicThreadTest, DestructStoppedThread)
{
	std::unique_ptr<MockThread> m;
	m.reset(new MockThread());

	m->start(nullptr);


	m->stop();

	EXPECT_NO_THROW(m.release());
}

TEST(BasicThreadTest, SynchronousThread)
{
	MockThread m(10);
	EXPECT_EQ(10, m.count);
	std::shared_ptr<Barrier> barrier = std::make_shared<Barrier>(2);
	EXPECT_FALSE(m.didInitialize);
	EXPECT_FALSE(m.didStartUp);
	EXPECT_FALSE(m.isRunning());
	EXPECT_FALSE(m.isSynchronous());

	m.start(barrier, true);

	// Run through the initialization
	barrier->wait(true);
	barrier->wait(true);
	barrier->wait(true);

	EXPECT_TRUE(m.isRunning());
	EXPECT_TRUE(m.isSynchronous());

	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	// It's running but waiting for a new wait call
	EXPECT_EQ(9, m.count);
	barrier->wait(true);

	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_EQ(8, m.count);
	barrier->wait(true);

	barrier->wait(false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	m.stop();
}

TEST(BasicThreadTest, PauseResumeUpdateTest)
{
	MockThread m(100000000);
	int previousCount = m.count;
	std::shared_ptr<Barrier> barrier = std::make_shared<Barrier>(2);

	EXPECT_EQ(100000000, m.count);
	EXPECT_FALSE(m.didInitialize);
	EXPECT_FALSE(m.didStartUp);
	EXPECT_FALSE(m.isRunning());
	EXPECT_FALSE(m.isSynchronous());
	EXPECT_FALSE(m.isIdle());

	m.start(barrier, true);

	// Run through the initialization
	barrier->wait(true);
	barrier->wait(true);
	barrier->wait(true);

	EXPECT_FALSE(m.isIdle());
	EXPECT_TRUE(m.isRunning());
	EXPECT_TRUE(m.isSynchronous());

	for (int i = 0; i < 10; i++)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		EXPECT_LT(m.count, previousCount);
		previousCount = m.count;
		barrier->wait(true);
	}

	m.setIdle(true);

	EXPECT_TRUE(m.isIdle());
	EXPECT_TRUE(m.isRunning());

	barrier->wait(true);
	previousCount = m.count;

	for (int i = 0; i < 10; i++)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		EXPECT_EQ(previousCount, m.count);
		previousCount = m.count;
		barrier->wait(true);
	}

	m.setIdle(false);

	EXPECT_FALSE(m.isIdle());
	EXPECT_TRUE(m.isRunning());

	for (int i = 0; i < 10; i++)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		EXPECT_LT(m.count, previousCount);
		previousCount = m.count;
		barrier->wait(true);
	}

	barrier->wait(false);
	boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	m.stop();
}

TEST(BasicThreadTest, SwitchSyncOnThread)
{
	MockThread m(-1);
	std::shared_ptr<Barrier> barrier = std::make_shared<Barrier>(2);
	EXPECT_FALSE(m.didInitialize);
	EXPECT_FALSE(m.didStartUp);
	EXPECT_FALSE(m.isRunning());
	EXPECT_FALSE(m.isSynchronous());

	m.start(barrier, false);

	// Run through the initialization
	barrier->wait(true);
	barrier->wait(true);

	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_TRUE(m.isRunning());

	// Thread is running, count should be less than count after waiting
	int count = m.count;
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_GT(count, m.count);

	m.setSynchronous(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));

	// Thread is stopped count should be equal to count after waiting
	count = m.count;
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_EQ(count, m.count);

	// Take one step, count should be just on less than last count
	barrier->wait(true);
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	EXPECT_EQ(count-1, m.count);

	count = count - 1;
	m.setSynchronous(false);
	// restart the thread ...
	barrier->wait(true);

	// Thread is running count should decrease and keep decreasing
	count = m.count;
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));
	EXPECT_GT(count, m.count);

	// Thread is running count should decrease and keep decreasing
	count = m.count;
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));
	EXPECT_GT(count, m.count);

	m.stop();
}

TEST(BasicThreadTest, RealTimings)
{
	MockThread m;
	m.start(nullptr);

	while(m.getTimer().getCumulativeTime() < 0.00001);
	EXPECT_GT(m.getTimer().getCumulativeTime(), 0.00001);
	EXPECT_GT(m.getTimer().getAverageFrameRate(), 0.0);

	m.stop();
}

// HS-2013-jun-25 Can't figure out how to make this work or what is going wrong with the test
class BasicThreadDeathTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m = std::make_shared<MockThread>();
		m->start(nullptr);
	}

	void TearDown()
	{
		m->stop();
	}

	std::shared_ptr<MockThread> m;
};

TEST_F(BasicThreadDeathTest, DestructLiveThread)
{
	::testing::FLAGS_gtest_death_test_style = "threadsafe";
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	ASSERT_DEATH_IF_SUPPORTED({
		SurgSim::Framework::AssertMessage::setFailureBehaviorToDeath();
		m.reset();
	}, "");
}

}; // namespace Framework
}; // namespace SurgSim
