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
/// Tests for the LockedContainer class.

#include <gtest/gtest.h>
#include "SurgSim/Framework/LockedContainer.h"

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

using SurgSim::Framework::LockedContainer;


// Define some helper data types for testing.

/// A class that supports copy construction and copy assignment but not moving.
class Copyable
{
public:
	Copyable() : m_data(-1)
	{
	}

	Copyable(const Copyable& o) : m_data(o.m_data)
	{
	}

	void operator=(const Copyable& o)
	{
		m_data = o.m_data;
	}

	int getValue() const
	{
		return m_data;
	}
	void setValue(int d)
	{
		m_data = d;
	}

private:
	// No need to disable move construction and move assignment-- the compiler does not provide those by default.

	int m_data;
};

/// A class that supports neither copying nor moving.
class NonCopyable
{
public:
	NonCopyable() : m_data(-1)
	{
	}

	int getValue() const
	{
		return m_data;
	}
	void setValue(int d)
	{
		m_data = d;
	}

private:
	// Disable copy construction and copy assignment.
	// No need to disable move construction and move assignment-- the compiler does not provide those by default.
	NonCopyable(const NonCopyable&);
	void operator=(const NonCopyable&);

	int m_data;
};

/// A class that supports move construction and move assignment but not copying.
class Movable
{
public:
	Movable() : m_data(-1)
	{
	}

	Movable(Movable&& o) : m_data(o.m_data)
	{
		o.m_data = -1;
	}
	void operator=(Movable&& o)
	{
		m_data = o.m_data;
		o.m_data = -1;
	}

	int getValue() const
	{
		return m_data;
	}
	void setValue(int d)
	{
		m_data = d;
	}

private:
	// Disable copy construction and copy assignment.
	Movable(const Movable&);
	void operator=(const Movable&);

	int m_data;
};

/// A class with several pieces of data for checking consistency.
class BigData
{
public:
	BigData() : m_data1(-1), m_data2(-1)
	{
	}

	BigData(const BigData& o) : m_data1(o.m_data1), m_data2(o.m_data2) {}
	void operator=(const BigData& o)
	{
		m_data1 = o.m_data1;
		m_data2 = o.m_data2;
	}

	BigData(BigData&& o) : m_data1(o.m_data1), m_data2(o.m_data2)
	{
		o.m_data1 = o.m_data2 = -1;
	}
	void operator=(BigData&& o)
	{
		m_data1 = o.m_data1;
		m_data2 = o.m_data2;
		o.m_data1 = o.m_data2 = -1;
	}

	int getValue1() const
	{
		return m_data1;
	}
	void setValue1(int d)
	{
		m_data1 = d;
	}
	int getValue2() const
	{
		return m_data2;
	}
	void setValue2(int d)
	{
		m_data2 = d;
	}

private:
	int m_data1;
	int padding[1023];  // make it less likely that the cache will be kind to us
	int m_data2;
};


// Now we're ready to start testing...

// ==================== SINGLE-THREADED TESTS ====================

TEST(LockedContainerTest, Construct)
{
	// This should work:
	EXPECT_NO_THROW( {LockedContainer<int> data;});
	EXPECT_NO_THROW( {LockedContainer<Copyable> data;});
	EXPECT_NO_THROW( {LockedContainer<Movable> data;});

	// This should also work but it's useless (can't modify the data):
	EXPECT_NO_THROW( {LockedContainer<NonCopyable> data;});
}

TEST(LockedContainerTest, InitializeAtConstruction)
{
	typedef Copyable DataType;
	// With the following definition, the container construction should not compile:
	//typedef NonCopyable DataType;

	DataType initial;
	initial.setValue(123);
	EXPECT_EQ(123, initial.getValue());

	LockedContainer<DataType> data(initial);

	DataType contents;
	data.get(&contents);
	EXPECT_EQ(123, contents.getValue());
	EXPECT_EQ(123, initial.getValue());
}

TEST(LockedContainerTest, MoveInitializeAtConstruction)
{
	typedef Movable DataType;

	DataType initial;
	initial.setValue(123);
	EXPECT_EQ(123, initial.getValue());

	LockedContainer<DataType> data(std::move(initial));

	DataType contents;
	data.take(&contents);
	EXPECT_EQ(123, contents.getValue());
	EXPECT_EQ(-1, initial.getValue());  // the old data has been moved out!
}

TEST(LockedContainerTest, Get)
{
	Copyable content;
	content.setValue(987);

	LockedContainer<Copyable> data(content);
	EXPECT_EQ(987, content.getValue());

	Copyable destination;
	data.get(&destination);
	EXPECT_EQ(987, destination.getValue());
	data.get(&destination);
	EXPECT_EQ(987, destination.getValue());
	EXPECT_EQ(987, content.getValue());
}

TEST(LockedContainerTest, Take)
{
	Movable content;
	content.setValue(987);

	LockedContainer<Movable> data(std::move(content));
	EXPECT_EQ(-1, content.getValue());

	Movable destination;
	data.take(&destination);
	EXPECT_EQ(987, destination.getValue());
	data.take(&destination);
	EXPECT_EQ(-1, destination.getValue());
	EXPECT_EQ(-1, content.getValue());
}

TEST(LockedContainerTest, SetAndGet)
{
	Copyable initial;
	initial.setValue(1);

	LockedContainer<Copyable> data(initial);

	Copyable contents;
	data.get(&contents);
	EXPECT_EQ(1, contents.getValue());

	// "writer":
	initial.setValue(2);
	// note: NOT writing to the container!

	// "reader":
	data.get(&contents);
	EXPECT_EQ(1, contents.getValue());

	// "writer":
	initial.setValue(3);
	data.set(initial);

	// "reader":
	data.get(&contents);
	EXPECT_EQ(3, contents.getValue());
}

TEST(LockedContainerTest, SetAndTake)
{
	Movable initial;
	initial.setValue(1);
	EXPECT_EQ(1, initial.getValue());

	LockedContainer<Movable> data(std::move(initial));

	Movable contents;
	data.take(&contents);
	EXPECT_EQ(1, contents.getValue());
	data.take(&contents);
	EXPECT_EQ(-1, contents.getValue());
	EXPECT_EQ(-1, initial.getValue());

	// "writer":
	initial.setValue(2);
	// note: NOT writing to the container!

	// "reader":
	data.take(&contents);
	EXPECT_EQ(-1, contents.getValue());

	// "writer":
	initial.setValue(3);
	data.set(std::move(initial));

	// "reader":
	data.take(&contents);
	EXPECT_EQ(3, contents.getValue());
	data.take(&contents);
	EXPECT_EQ(-1, contents.getValue());
	EXPECT_EQ(-1, initial.getValue());
}

TEST(LockedContainerTest, TryGetChanged)
{
	{
		LockedContainer<Copyable> data;  // use default constructor for Copyable

		Copyable destination;
		destination.setValue(999);
		{
			bool changed = data.tryGetChanged(&destination);
			EXPECT_FALSE(changed) << "no set(); tryGetChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(999, destination.getValue()) <<
					"tryGetChanged() returned false; value shouldn't have changed!";
			}
		}

		Copyable content;
		content.setValue(101);
		data.set(content);

		{
			bool changed = data.tryGetChanged(&destination);
			EXPECT_TRUE(changed) << "set() called; tryGetChanged() should return true!";
			if (changed)
			{
				EXPECT_EQ(101, destination.getValue()) <<
					"tryGetChanged() returned true; value should have changed!";
			}
		}

		destination.setValue(888);
		{
			bool changed = data.tryGetChanged(&destination);
			EXPECT_FALSE(changed) << "no set() since get(); tryGetChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(888, destination.getValue()) <<
					"tryGetChanged() returned false; value shouldn't have changed!";
			}
		}
	}

	{
		Copyable content;
		content.setValue(654);
		LockedContainer<Copyable> data(content);  // use initializing constructor for Copyable

		Copyable destination;
		destination.setValue(999);
		{
			bool changed = data.tryGetChanged(&destination);
			EXPECT_FALSE(changed) << "no set(); tryGetChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(999, destination.getValue()) <<
					"tryGetChanged() returned false; value shouldn't have changed!";
			}
		}

		content.setValue(121);
		data.set(content);

		{
			bool changed = data.tryGetChanged(&destination);
			EXPECT_TRUE(changed) << "set() called; tryGetChanged() should return true!";
			if (changed)
			{
				EXPECT_EQ(121, destination.getValue()) <<
					"tryGetChanged() returned true; value should have changed!";
			}
		}

		destination.setValue(888);
		{
			bool changed = data.tryGetChanged(&destination);
			EXPECT_FALSE(changed) << "no set() since get(); tryGetChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(888, destination.getValue()) <<
					"tryGetChanged() returned false; value shouldn't have changed!";
			}
		}
	}
}

TEST(LockedContainerTest, TryTakeChanged)
{
	{
		LockedContainer<Movable> data;  // use default constructor for Movable

		Movable destination;
		destination.setValue(999);
		{
			bool changed = data.tryTakeChanged(&destination);
			EXPECT_FALSE(changed) << "no set(); tryTakeChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(999, destination.getValue()) <<
					"tryTakeChanged() returned false; value shouldn't have changed!";
			}
		}

		Movable content;
		content.setValue(101);
		data.set(std::move(content));

		{
			bool changed = data.tryTakeChanged(&destination);
			EXPECT_TRUE(changed) << "set() called; tryTakeChanged() should return true!";
			if (changed)
			{
				EXPECT_EQ(101, destination.getValue()) <<
					"tryTakeChanged() returned true; value should have changed!";
			}
		}

		destination.setValue(888);
		{
			bool changed = data.tryTakeChanged(&destination);
			EXPECT_FALSE(changed) << "no set() since get(); tryTakeChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(888, destination.getValue()) <<
					"tryTakeChanged() returned false; value shouldn't have changed!";
			}
		}

		destination.setValue(777);
		{
			data.take(&destination);
			EXPECT_EQ(-1, destination.getValue()) <<
				"an earlier tryTakeChanged() should have emptied the container!";
		}
	}

	{
		Movable content;
		content.setValue(654);
		LockedContainer<Movable> data(std::move(content));  // use initializing constructor for Movable

		Movable destination;
		destination.setValue(999);
		{
			bool changed = data.tryTakeChanged(&destination);
			EXPECT_FALSE(changed) << "no set(); tryTakeChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(999, destination.getValue()) <<
					"tryTakeChanged() returned false; value shouldn't have changed!";
			}
		}

		content.setValue(121);
		data.set(std::move(content));

		{
			bool changed = data.tryTakeChanged(&destination);
			EXPECT_TRUE(changed) << "set() called; tryTakeChanged() should return true!";
			if (changed)
			{
				EXPECT_EQ(121, destination.getValue()) <<
					"tryTakeChanged() returned true; value should have changed!";
			}
		}

		destination.setValue(888);
		{
			bool changed = data.tryTakeChanged(&destination);
			EXPECT_FALSE(changed) << "no set() since get(); tryTakeChanged() should return false!";
			if (! changed)
			{
				EXPECT_EQ(888, destination.getValue()) <<
					"tryTakeChanged() returned false; value shouldn't have changed!";
			}
		}

		destination.setValue(777);
		{
			data.take(&destination);
			EXPECT_EQ(-1, destination.getValue()) <<
				"an earlier tryTakeChanged() should have emptied the container!";
		}
	}
}

// ==================== MULTI-THREADED TESTS ====================

typedef LockedContainer<BigData> SharedData;

class DataWriter
{
public:
	/// data parameter to be passed by non-const reference on purpose.
	explicit DataWriter(SharedData& data, int start = 0, int step = 1, int loops = 100000) : //NOLINT
		m_data(data),
		m_start(start),
		m_step(step),
		m_loops(loops),
		m_isDone(false)
	{
	}

	bool isDone() const
	{
		// Direct access to the data member is a hack, since this is called from a different thread.
		// If this were production code, we would really want to use a LockedContainer<bool> instead.
		//
		// But in this case, we really want to avoid locking around the flag in order to maximize the likelihood
		// that adverse race conditions in *actual data* access may pop up, so we do this anyway.
		return m_isDone;
	}

	void start()
	{
		m_thread = boost::thread(boost::ref(*this));
	}
	void join()
	{
		m_thread.join();
	}

	void operator()()
	{
		BigData current;

		int value = m_start;
		while (m_loops > 0)
		{
			--m_loops;
			value += m_step;

			current.setValue1(value);
			current.setValue2(value);
			m_data.set(current);
		}

		m_isDone = true;
	}

private:
	DataWriter(const DataWriter&);
	void operator=(const DataWriter&);

	SharedData& m_data;
	int m_start;
	int m_step;
	int m_loops;
	bool m_isDone;

	boost::thread m_thread;
};

void testReaderAndWriters(int numWriters)
{
	// The number of total writes performed by all writers.
	//
	// Note that this value shouldn't be too low: when you break the code by eliminating locking altogether, you
	// don't get reliable failures below about 10000 writes (as tested on a MSI GE70 laptop), presumably because
	// the threads finish too quickly and thus don't run concurrently.
	const int NUM_TOTAL_WRITES = 1000000;

	LockedContainer<BigData> data;

	std::vector<std::unique_ptr<DataWriter>> writers(numWriters);
	for (size_t i = 0;  i < writers.size();  ++i)
	{
		// The step has been chosen so two writers can't ever produce the same value
		writers[i] = std::unique_ptr<DataWriter>(new DataWriter(data, static_cast<int>(i), numWriters,
			NUM_TOTAL_WRITES / numWriters));
	}
	{
		BigData value;
		data.get(&value);
		EXPECT_EQ(-1, value.getValue1());
		EXPECT_EQ(-1, value.getValue2());
	}

	for (size_t i = 0;  i < writers.size();  ++i)
	{
		writers[i]->start();
	}

	BigData value;
	while (1)
	{
		int z1 = value.getValue1();

		bool wasUpdated = data.tryGetChanged(&value);
		int a1 = value.getValue1();
		int a2 = value.getValue2();
		EXPECT_EQ(a1, a2);
		if (wasUpdated)
		{
			EXPECT_NE(z1, a1);
		}
		else
		{
			EXPECT_EQ(z1, a1);
		}

		bool allDone = true;
		for (size_t i = 0;  i < writers.size();  ++i)
		{
			if (! writers[i]->isDone())
			{
				allDone = false;
				break;
			}
		}
		if (allDone)
		{
			break;
		}
	}

	for (size_t i = 0;  i < writers.size();  ++i)
	{
		writers[i]->join();
	}
}

TEST(LockedContainerTest, DISABLED_OneWriterThread)
{
	testReaderAndWriters(1);
}
TEST(LockedContainerTest, TwoWriterThreads)
{
	testReaderAndWriters(2);
}
TEST(LockedContainerTest, DISABLED_FourWriterThreads)
{
	testReaderAndWriters(4);
}
TEST(LockedContainerTest, DISABLED_EightWriterThreads)
{
	testReaderAndWriters(8);
}
TEST(LockedContainerTest, SixteenWriterThreads)
{
	testReaderAndWriters(16);
}
