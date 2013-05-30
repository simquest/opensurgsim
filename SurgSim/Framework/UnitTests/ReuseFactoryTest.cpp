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

#include <SurgSim/Framework/ReuseFactory.h>

using SurgSim::Framework::ReuseFactory;

class MockObject
{
public:
	MockObject() {};
	~MockObject() {};
};

TEST(ReuseFactoryTest, InitTest)
{
	ASSERT_NO_THROW({ReuseFactory<MockObject>();});
}

TEST(ReuseFactoryTest, GetNewTest)
{
	ReuseFactory<MockObject> objectFactory;

	std::shared_ptr<MockObject> object = objectFactory.getInstance();
	ASSERT_NE(nullptr, object);
}

TEST(ReuseFactoryTest, DeleteTest)
{
	ReuseFactory<MockObject> objectFactory;

	std::weak_ptr<MockObject> weakPointer;
	{
		std::shared_ptr<MockObject> sharedPointer;
		{
			// Get a new object.
			std::shared_ptr<MockObject> object = objectFactory.getInstance();
			ASSERT_NE(nullptr, object);

			weakPointer = object;
			ASSERT_NE(nullptr, weakPointer.lock());

			// Make another shared pointer to the object.
			sharedPointer = object;
		}

		// The object should not have been deleted yet, as there is still a shared pointer to it.
		ASSERT_NE(nullptr, weakPointer.lock());
	}

	// It should be deleted now.
	EXPECT_EQ(nullptr, weakPointer.lock()) << "The object should no longer exist.";
}

TEST(ReuseFactoryTest, ReuseTest)
{
	ReuseFactory<MockObject> objectFactory;

	std::weak_ptr<MockObject> weakPointer;
	MockObject* pointer;

	// Get a new object
	{
		std::shared_ptr<MockObject> object = objectFactory.getInstance();
		ASSERT_NE(nullptr, object);

		weakPointer = object;
		ASSERT_NE(nullptr, weakPointer.lock());

		pointer = object.get();
	}

	// It should be deleted now.
	EXPECT_EQ(nullptr, weakPointer.lock()) << "Weak pointer to object = " << weakPointer.lock();
	
	// Reuse the object that has been deleted.
	{
		std::shared_ptr<MockObject> object = objectFactory.getInstance();
		ASSERT_NE(nullptr, object);

		EXPECT_EQ(pointer, object.get()) << "The retrieved object should be the object that was previously deleted.";

		// Get another object, which should be new.
		{
			std::shared_ptr<MockObject> object = objectFactory.getInstance();
			ASSERT_NE(nullptr, object);

			EXPECT_NE(pointer, object.get()) << "We should have a completely new object, not the same previous object.";
		}
	}
}
