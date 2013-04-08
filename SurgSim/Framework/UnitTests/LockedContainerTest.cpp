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

/** @file
 * Tests for the ThreadSafeContainer class.
 */

#include <gtest/gtest.h>
#include <SurgSim/Framework/ThreadSafeContainer.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

using SurgSim::Framework::ThreadSafeContainer;


// Define some helper data types for testing.

class Copyable
{
public:
	Copyable() : m_data(-1) {};
	Copyable(const Copyable& o) : m_data(o.m_data) {}
	void operator=(const Copyable& o) { m_data = o.m_data; }

	int getValue() const { return m_data; }
	void setValue(int d) { m_data = d; }

private:
	int m_data;
};

class NonCopyable
{
public:
	NonCopyable() : m_data(-1) {};

	int getValue() const { return m_data; }
	void setValue(int d) { m_data = d; }

private:
	NonCopyable(const NonCopyable&);
	void operator=(const NonCopyable&);

	int m_data;
};

class Movable
{
public:
	Movable() : m_data(-1) {};
	Movable(Movable&& o) : m_data(o.m_data) { o.m_data = -1; }
	void operator=(Movable&& o) { m_data = o.m_data;  o.m_data = -1; }

	int getValue() const { return m_data; }
	void setValue(int d) { m_data = d; }

private:
	Movable(const Movable&);
	void operator=(const Movable&);

	int m_data;
};

class BigData
{
public:
	BigData() : m_data1(-1), m_data2(-1) {};

	BigData(const BigData& o) : m_data1(o.m_data1), m_data2(o.m_data2) {}
	void operator=(const BigData& o) { m_data1 = o.m_data1;  m_data2 = o.m_data2; }

	BigData(BigData&& o) : m_data1(o.m_data1), m_data2(o.m_data2) { o.m_data1 = o.m_data2 = -1; }
	void operator=(BigData&& o) { m_data1 = o.m_data1;  m_data2 = o.m_data2;  o.m_data1 = o.m_data2 = -1; }

	int getValue1() const { return m_data1; }
	void setValue1(int d) { m_data1 = d; }
	int getValue2() const { return m_data2; }
	void setValue2(int d) { m_data2 = d; }

private:
	int m_data1;
	int padding[1023];  // make it less likely that the cache will be kind to us
	int m_data2;
};

// Now we're ready to start testing...

// ==================== SINGLE-THREADED TESTS ====================

TEST(ThreadSafeContainerTest, Construct)
{
	// This should work:
	EXPECT_NO_THROW({ThreadSafeContainer<int> data;});
	EXPECT_NO_THROW({ThreadSafeContainer<Copyable> data;});
	EXPECT_NO_THROW({ThreadSafeContainer<Movable> data;});

	// This should also work but it's useless (can't modify the data):
	EXPECT_NO_THROW({ThreadSafeContainer<NonCopyable> data;});
}

TEST(ThreadSafeContainerTest, InitializeAtConstruction)
{
	typedef Copyable DataType;
	// With the following definition, the TSC construction should not compile:
	//typedef NonCopyable DataType;

	DataType initial;
	initial.setValue(123);
	EXPECT_EQ(123, initial.getValue());

	ThreadSafeContainer<DataType> data(initial);
	EXPECT_EQ(123, data->getValue());
	EXPECT_EQ(123, initial.getValue());
}

TEST(ThreadSafeContainerTest, MoveInitializeAtConstruction)
{
	typedef Movable DataType;

	DataType initial;
	initial.setValue(123);
	EXPECT_EQ(123, initial.getValue());

	ThreadSafeContainer<DataType> data(std::move(initial));
	EXPECT_EQ(123, data->getValue());
	EXPECT_EQ(-1, initial.getValue());
}

TEST(ThreadSafeContainerTest, ReadWriteInterlock)
{
	Copyable content;
	content.setValue(1);

	ThreadSafeContainer<Copyable> data(content);
	EXPECT_EQ(1, data->getValue());
	bool wasUpdated = data.update();
	EXPECT_FALSE(wasUpdated);

	// "writer":
	content.setValue(2);
	// note: NOT writing to the container!

	// "reader":
	wasUpdated = data.update();
	EXPECT_FALSE(wasUpdated);
	EXPECT_EQ(1, data->getValue());

	// "writer":
	content.setValue(3);
	data.set(content);

	// "reader":
	// note: NOT calling data.update()!
	EXPECT_EQ(1, data->getValue());

	// "writer": reusing previously written value

	// "reader":
	wasUpdated = data.update();
	EXPECT_TRUE(wasUpdated);
	EXPECT_EQ(3, data->getValue());
}

TEST(ThreadSafeContainerTest, MoveInterfaces)
{
	Movable content;
	content.setValue(1);

	ThreadSafeContainer<Movable> data(std::move(content));
	EXPECT_EQ(1, data->getValue());
	bool wasUpdated = data.update();
	EXPECT_FALSE(wasUpdated);

	// "writer":
	content.setValue(2);
	// note: NOT writing to the container!

	// "reader":
	wasUpdated = data.update();
	EXPECT_FALSE(wasUpdated);
	EXPECT_EQ(1, data->getValue());

	// "writer":
	content.setValue(3);
	data.set(std::move(content));

	// "reader":
	// note: NOT calling data.update()!
	EXPECT_EQ(1, data->getValue());

	// "writer": reusing previously written value

	// "reader":
	wasUpdated = data.update();
	EXPECT_TRUE(wasUpdated);
	EXPECT_EQ(3, data->getValue());
}

// ==================== MULTI-THREADED TESTS ====================

typedef ThreadSafeContainer<BigData> SharedData;

class DataWriter
{
public:
	explicit DataWriter(SharedData& data, int start = 0, int step = 1, int loops = 100000) :
		m_data(data),
		m_start(start),
		m_step(step),
		m_loops(loops),
		m_isDone(false)
	{
	}

	bool isDone() const
	{
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

	ThreadSafeContainer<BigData> data;

	std::vector<DataWriter*> writers(numWriters);
	for (size_t i = 0;  i < writers.size();  ++i)
	{
		// The step has been chosen so two writers can't ever produce the same value
		writers[i] = new DataWriter(data, i, numWriters, NUM_TOTAL_WRITES/numWriters);
	}
	data.update();
	EXPECT_EQ(-1, data->getValue1());
	EXPECT_EQ(-1, data->getValue2());

	for (size_t i = 0;  i < writers.size();  ++i)
	{
		writers[i]->start();
	}

	while (1)
	{
		int z1 = data->getValue1();

		bool wasUpdated = data.update();
		int a1 = data->getValue1();
		int a2 = data->getValue2();
		EXPECT_EQ(a1, a2);
		if (wasUpdated)
		{
			EXPECT_NE(z1, a1);
		}
		else
		{
			EXPECT_EQ(z1, a1);
		}

		int b1 = data->getValue1();
		int b2 = data->getValue2();
		EXPECT_EQ(a1, b1);
		EXPECT_EQ(b1, b2);

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

TEST(ThreadSafeContainerTest, DISABLED_OneWriterThread)
{
	testReaderAndWriters(1);
}
TEST(ThreadSafeContainerTest, TwoWriterThreads)
{
	testReaderAndWriters(2);
}
TEST(ThreadSafeContainerTest, DISABLED_FourWriterThreads)
{
	testReaderAndWriters(4);
}
TEST(ThreadSafeContainerTest, DISABLED_EightWriterThreads)
{
	testReaderAndWriters(8);
}
TEST(ThreadSafeContainerTest, SixteenWriterThreads)
{
	testReaderAndWriters(16);
}
