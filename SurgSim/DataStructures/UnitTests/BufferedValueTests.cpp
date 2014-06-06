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

TEST(BufferedValueTests, InitTest)
{
	EXPECT_NO_THROW({BufferedValue<int> value(3);});
	EXPECT_NO_THROW({BufferedValue<int> value;});
}

TEST(BufferedValueTests, SafeAndUnsafeGettersTest)
{
	auto buffer = std::make_shared<BufferedValue<int>>(10);
	int& value = buffer->unsafeGet();
	std::shared_ptr<const int> initialBufferedValue = buffer->safeGet();

	EXPECT_EQ(10, value);
	EXPECT_EQ(10, *initialBufferedValue);

	value = 20;
	EXPECT_EQ(20, value);
	EXPECT_EQ(10, *initialBufferedValue);

	std::shared_ptr<const int> postAssignBufferedValue = buffer->safeGet();
	EXPECT_EQ(initialBufferedValue, postAssignBufferedValue);

	buffer->publish();

	EXPECT_EQ(10, *initialBufferedValue);

	std::shared_ptr<const int> postPublishBufferedValue = buffer->safeGet();
	EXPECT_EQ(20, *postPublishBufferedValue);
}


}
}

