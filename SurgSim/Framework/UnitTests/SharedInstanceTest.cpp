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

/// \file
/// Tests for the SharedInstance class.

#include <type_traits>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <gtest/gtest.h>
#include "SurgSim/Framework/SharedInstance.h"

using SurgSim::Framework::SharedInstance;


// Define some helper data types for testing.

/// A helper base class.
class ObjectCounts
{
public:
	static int getNumInstancesCreated()
	{
		return m_constructionIndex;
	}
	static int getNumInstancesDestroyed()
	{
		return m_destructionIndex;
	}

	static void resetInstanceIndices()
	{
		m_constructionIndex = 0;
		m_destructionIndex = 0;
	}

protected:
	ObjectCounts()
	{
	}

	static int getNextInstanceIndex()
	{
		// NB: not thread-safe!
		++m_constructionIndex;
		return m_constructionIndex;
	}

	static void markDestruction()
	{
		// NB: not thread-safe!
		++m_destructionIndex;
	}

	static int m_constructionIndex;
	static int m_destructionIndex;
};

int ObjectCounts::m_constructionIndex = 0;
int ObjectCounts::m_destructionIndex = 0;


/// A class that only supports construction with an argument, and doesn't support copying or moving.
class ArgumentOnlyConstructible : public ObjectCounts
{
public:
	explicit ArgumentOnlyConstructible(int data) : m_data(data), m_instanceIndex(getNextInstanceIndex())
	{
	}

	~ArgumentOnlyConstructible()
	{
		markDestruction();
	}

	int getValue() const
	{
		return m_data;
	}
	void setValue(int d)
	{
		m_data = d;
	}

	int getInstanceIndex() const
	{
		return m_instanceIndex;
	}

private:
	// Disable copy construction and copy assignment.
	// No need to disable move construction and move assignment-- the compiler does not provide those by default.
	ArgumentOnlyConstructible(const ArgumentOnlyConstructible&);
	void operator=(const ArgumentOnlyConstructible&);

	int m_data;
	int m_instanceIndex;
};


/// A class that supports default construction, but neither copying nor moving.
class DefaultConstructible : public ArgumentOnlyConstructible
{
public:
	DefaultConstructible() : ArgumentOnlyConstructible(-1)
	{
	}
};


// Now we're ready to start testing...

// ==================== SINGLE-THREADED TESTS ====================

TEST(SharedInstanceTest, Construct)
{
	ObjectCounts::resetInstanceIndices();

	// This should work:
	EXPECT_NO_THROW({SharedInstance<int> shared;});
	EXPECT_NO_THROW({SharedInstance<DefaultConstructible> shared;});

	// This should NOT work:
//	EXPECT_NO_THROW({SharedInstance<ArgumentOnlyConstructible> shared;});
	// ...but this should:
	SharedInstance<ArgumentOnlyConstructible>::InstanceCreator createArgumentOnlyConstructible =
		[]() { return std::make_shared<ArgumentOnlyConstructible>(1); };
	EXPECT_NO_THROW({SharedInstance<ArgumentOnlyConstructible> shared(createArgumentOnlyConstructible);});

	// No objects should have been constructed:
	EXPECT_EQ(0, ObjectCounts::getNumInstancesCreated());
}

TEST(SharedInstanceTest, CreateInstance)
{
	typedef DefaultConstructible DataType;

	ObjectCounts::resetInstanceIndices();

	SharedInstance<DataType> shared;
	EXPECT_EQ(0, ObjectCounts::getNumInstancesCreated());

	{
		std::shared_ptr<DataType> ref1 = shared.get();
		ASSERT_NE(nullptr, ref1);
		EXPECT_EQ(1, ref1->getInstanceIndex());
		std::shared_ptr<DataType> ref2 = shared.get();
		ASSERT_NE(nullptr, ref2);
		EXPECT_EQ(1, ref2->getInstanceIndex());
		EXPECT_EQ(ref1, ref2);
		EXPECT_EQ(1, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(0, ObjectCounts::getNumInstancesDestroyed());
	}
	EXPECT_EQ(1, ObjectCounts::getNumInstancesCreated());
	EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());

	{
		std::shared_ptr<DataType> ref1 = shared.get();
		std::shared_ptr<DataType> ref2 = shared.get();
		EXPECT_EQ(ref1, ref2);
		EXPECT_EQ(2, ref1->getInstanceIndex());
		EXPECT_EQ(2, ref2->getInstanceIndex());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());

		ref1.reset();
		ASSERT_EQ(nullptr, ref1);  // it's dead now!
		ASSERT_NE(nullptr, ref2);
		EXPECT_EQ(2, ref2->getInstanceIndex());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());
		ref2.reset();
		ASSERT_EQ(nullptr, ref2);  // it too is dead now!
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesDestroyed());
	}
	EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
	EXPECT_EQ(2, ObjectCounts::getNumInstancesDestroyed());
}

TEST(SharedInstanceTest, CreateInstanceUsingCreator)
{
	typedef DefaultConstructible DataType;

	ObjectCounts::resetInstanceIndices();

	SharedInstance<DataType>::InstanceCreator createDataType( []() { return std::make_shared<DataType>(); } );
	SharedInstance<DataType> shared(createDataType);

	EXPECT_EQ(0, ObjectCounts::getNumInstancesCreated());

	{
		std::shared_ptr<DataType> ref1 = shared.get();
		ASSERT_NE(nullptr, ref1);
		EXPECT_EQ(1, ref1->getInstanceIndex());
		std::shared_ptr<DataType> ref2 = shared.get();
		ASSERT_NE(nullptr, ref2);
		EXPECT_EQ(1, ref2->getInstanceIndex());
		EXPECT_EQ(ref1, ref2);
		EXPECT_EQ(1, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(0, ObjectCounts::getNumInstancesDestroyed());
	}
	EXPECT_EQ(1, ObjectCounts::getNumInstancesCreated());
	EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());

	{
		std::shared_ptr<DataType> ref1 = shared.get();
		std::shared_ptr<DataType> ref2 = shared.get();
		EXPECT_EQ(ref1, ref2);
		EXPECT_EQ(2, ref1->getInstanceIndex());
		EXPECT_EQ(2, ref2->getInstanceIndex());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());

		ref1.reset();
		ASSERT_EQ(nullptr, ref1);  // it's dead now!
		ASSERT_NE(nullptr, ref2);
		EXPECT_EQ(2, ref2->getInstanceIndex());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());
		ref2.reset();
		ASSERT_EQ(nullptr, ref2);  // it too is dead now!
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesDestroyed());
	}
	EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
	EXPECT_EQ(2, ObjectCounts::getNumInstancesDestroyed());
}

TEST(SharedInstanceTest, CreateNonDefaultConstructibleInstance)
{
	typedef ArgumentOnlyConstructible DataType;

	ObjectCounts::resetInstanceIndices();

	// This should NOT work for a type that's not default-constructible:
//	SharedInstance<DataType> shared;
	// ...but this should:
	SharedInstance<DataType>::InstanceCreator createDataType( []() { return std::make_shared<DataType>(2); } );
	SharedInstance<DataType> shared(createDataType);

	EXPECT_EQ(0, ObjectCounts::getNumInstancesCreated());

	{
		std::shared_ptr<DataType> ref1 = shared.get();
		ASSERT_NE(nullptr, ref1);
		EXPECT_EQ(1, ref1->getInstanceIndex());
		std::shared_ptr<DataType> ref2 = shared.get();
		ASSERT_NE(nullptr, ref2);
		EXPECT_EQ(1, ref2->getInstanceIndex());
		EXPECT_EQ(ref1, ref2);
		EXPECT_EQ(1, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(0, ObjectCounts::getNumInstancesDestroyed());
	}
	EXPECT_EQ(1, ObjectCounts::getNumInstancesCreated());
	EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());

	{
		std::shared_ptr<DataType> ref1 = shared.get();
		std::shared_ptr<DataType> ref2 = shared.get();
		EXPECT_EQ(ref1, ref2);
		EXPECT_EQ(2, ref1->getInstanceIndex());
		EXPECT_EQ(2, ref2->getInstanceIndex());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());

		ref1.reset();
		ASSERT_EQ(nullptr, ref1);  // it's dead now!
		ASSERT_NE(nullptr, ref2);
		EXPECT_EQ(2, ref2->getInstanceIndex());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(1, ObjectCounts::getNumInstancesDestroyed());
		ref2.reset();
		ASSERT_EQ(nullptr, ref2);  // it too is dead now!
		EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
		EXPECT_EQ(2, ObjectCounts::getNumInstancesDestroyed());
	}
	EXPECT_EQ(2, ObjectCounts::getNumInstancesCreated());
	EXPECT_EQ(2, ObjectCounts::getNumInstancesDestroyed());
}
