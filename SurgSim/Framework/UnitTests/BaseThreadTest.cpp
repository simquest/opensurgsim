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


#include <SurgSim/Framework/BasicThread.h>
#include "MockObjects.h"  //NOLINT

TEST(BasicThreadTest, Instantiation)
{
	MockThread m;
	EXPECT_FALSE(m.isInitialized());
	EXPECT_FALSE(m.isRunning());
}

TEST(BasicThreadTest, Running)
{
	MockThread m;
	m.start(nullptr);

	m.getThread().join();

	EXPECT_EQ(0, m.count);
}

TEST(ThreadTest, Stop)
{
	MockThread m;
	m.start(nullptr);

	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	m.stop();

	EXPECT_TRUE(m.didBeforeStop);
}
