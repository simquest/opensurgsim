// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest LLC.
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
#include "SurgSim/Framework/Barrier.h"
#include "MockObjects.h"

#include <boost/thread.hpp>

using SurgSim::Framework::Barrier;

namespace {
	std::shared_ptr<Barrier> barrier;
	void threadSuccessFunc()
	{
		barrier->wait(true);
	}

	void threadFailureFunc()
	{
		barrier->wait(false);
	}
}


TEST(BarrierTest, BasicTest)
{
	ASSERT_ANY_THROW({Barrier(0);});
	EXPECT_NO_THROW({Barrier(2);});
}

TEST(BarrierTest, SuccessTest)
{
	barrier = std::make_shared<Barrier>(2);
	boost::thread thread(threadSuccessFunc);
	EXPECT_TRUE(barrier->wait(true));
}

TEST(BarrierTest, FailureTest)
{
	barrier = std::make_shared<Barrier>(2);
	boost::thread thread(threadFailureFunc);
	EXPECT_FALSE(barrier->wait(true));
}
