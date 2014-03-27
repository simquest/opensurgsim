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


#include "SurgSim/DataStructures/BufferedValue.h"

#include <boost/thread.hpp>
#include <memory>

namespace SurgSim
{
namespace DataStructures
{

void wrongThreadWriterLock(std::shared_ptr<BufferedValue<int>> value, bool* exceptionOccured)
{
	*exceptionOccured = false;
	try
	{
		ReadWriteAccessor<int> writer(value);
	}
	catch (std::exception e)
	{
		*exceptionOccured = true;
	}
}

struct TestStruct
{
	int a;
	double b;
};

void functionTest(int value, int* testpointer)
{
	*testpointer = value;
}



TEST(BufferedValueTests, InitTest)
{

	EXPECT_NO_THROW({BufferedValue<int> value;});
}

TEST(BufferedValueTests, WriterLockTest)
{
	auto buffer = std::make_shared<BufferedValue<int>>(10);

	std::unique_ptr<ReadWriteAccessor<int>> accessor0;
	std::unique_ptr<ReadWriteAccessor<int>> accessor1;

	// First accessor, no writer, this should succeed
	EXPECT_NO_THROW(accessor0.reset(new ReadWriteAccessor<int>(buffer)));

	// Second accessor, should throw as there is another writer
	EXPECT_ANY_THROW(accessor1.reset(new ReadWriteAccessor<int>(buffer)));

	// Release accessor
	EXPECT_NO_THROW(accessor0.reset());

	bool hadException = false;
	boost::thread(wrongThreadWriterLock, buffer, &hadException);
	// Buffer was allocated in this thread, this should fail
	boost::this_thread::sleep(boost::posix_time::millisec(100));
	EXPECT_TRUE(hadException);

	// Should succeed again, if the previous release was correct
	EXPECT_NO_THROW(accessor0.reset(new ReadWriteAccessor<int>(buffer)));

}

TEST(BufferedValueTests, ReadWriteSimpleAccessorTest)
{
	auto buffer = std::make_shared<BufferedValue<int>>(10);
	ReadWriteAccessor<int> accessor(buffer);

	EXPECT_EQ(10, *accessor);

	*accessor = 20;

	EXPECT_EQ(20, *accessor);
}

TEST(BufferedValueTests, ReadWriteFunctionTest)
{
	auto buffer = std::make_shared<BufferedValue<int>>(10);
	ReadWriteAccessor<int> accessor(buffer);

	functionTest(20, accessor.get());

	EXPECT_EQ(20, *accessor);
}


TEST(BufferedValueTests, ReadWriteStructAccessorTest)
{
	auto buffer = std::make_shared<BufferedValue<TestStruct>>();
	ReadWriteAccessor<TestStruct> accessor(buffer);

	accessor->a = 10;
	accessor->b = 20.0;

	EXPECT_EQ(10, (*accessor).a);
	EXPECT_EQ(20.0, (*accessor).b);

	(*accessor).a = 5;
	(*accessor).b = 10.0;

	EXPECT_EQ(5, accessor->a);
	EXPECT_EQ(10.0, accessor->b);
}

TEST(BufferedValueTests, ReadAccessorTest)
{
	auto buffer = std::make_shared<BufferedValue<int>>(10);
	ReadWriteAccessor<int> readWriteValue(buffer);
	UnsafeAccessor<int> unsafeValue(buffer);
	SafeAccessor<int> safeValue(buffer);

	EXPECT_EQ(10, *unsafeValue);
	EXPECT_EQ(10, *safeValue);

	*readWriteValue = 20;

	EXPECT_EQ(20, *unsafeValue);
	EXPECT_EQ(10, *safeValue);

	EXPECT_NO_THROW(readWriteValue.publish());

	EXPECT_EQ(20, *unsafeValue);
	EXPECT_EQ(20, *safeValue);
}


}
}

