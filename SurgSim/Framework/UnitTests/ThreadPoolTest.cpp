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


#include "SurgSim/Framework/ThreadPool.h"

#include <gtest/gtest.h>

namespace
{

double f1() { return 1.0; }
int f2(int val) { return val; }

};

namespace SurgSim
{
namespace Framework
{

TEST(ThreadPoolTest, CanConstruct)
{
	EXPECT_NO_THROW({ThreadPool pool;});
	EXPECT_NO_THROW({ThreadPool pool(2);});
}

TEST(ThreadPoolTest, ExampleUsage)
{
	ThreadPool pool;

	// Add a task
	std::future<double> result1 = pool.enqueue<double>(f1);

	// Add a task using std::bind
	std::future<int> result2 = pool.enqueue<int>(std::bind(f2, 2));

	// Add a task using a lambda function
	std::future<std::string> result3 = pool.enqueue<std::string>([]() {return "string"; });

	EXPECT_EQ(1.0, result1.get());
	EXPECT_EQ(2, result2.get());
	EXPECT_EQ("string", result3.get());
}

TEST(ThreadPoolTest, AddLotsOfTasks)
{
	ThreadPool pool(2);

	std::vector<std::future<int>> futures;
	int expectedTotal = 0;
	for (int i = 0; i < 100; i++)
	{
		futures.push_back(pool.enqueue<int>(std::bind(f2, i)));
		expectedTotal += i;
	}

	int total = 0;
	for (auto& future : futures)
	{
		total += future.get();
	}
	EXPECT_EQ(expectedTotal, total);
}

};
};
