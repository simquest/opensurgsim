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

#ifndef SURGSIM_FRAMEWORK_THREADPOOL_H
#define SURGSIM_FRAMEWORK_THREADPOOL_H

#include <atomic>
#include <boost/thread.hpp>
#include <functional>
#include <future>
#include <memory>
#include <queue>
#include <vector>


namespace SurgSim
{
namespace Framework
{

/// A thread pool for completing heterogenous tasks
///
/// The thread pool is a class that completes given tasks using a set of worker
/// threads. These threads pull tasks off of a task queue. Once finished with
/// the task, the thread gets the next task if one is available, or waits for
/// another task to be added. The tasks can be heterogenous, meaning any
/// callable target can be added with any return type.
///
/// Example Usage:
/// \code{.cpp}
/// double f1() { return 1.0; }
/// int f2(int val) { return val; }
///
/// int main()
/// {
///		ThreadPool pool;
///
///		// Add a task
///		std::future<double> result1 = pool.enqueue<double>(f1);
///
///		// Add a task using std::bind
///		std::future<int> result2 = pool.enqueue<int>(std::bind(f2, 2));
///
///		// Add a task using a lambda function
///		std::future<std::string> result3 = pool.enqueue<std::string>([]() {return "string"; });
///
///		// Print out result when task is completed
///		std::cout << "Result 1: " << result1.get() << std::endl;
///		std::cout << "Result 2: " << result2.get() << std::endl;
///		std::cout << "Result 3: " << result3.get() << std::endl;
/// }
/// \endcode
class ThreadPool
{
public:
	/// Constructor
	/// \param numThreads The number of worker threads
	explicit ThreadPool(size_t numThreads = boost::thread::hardware_concurrency());

	/// Desctructor
	~ThreadPool();

	/// Queue a task to be run by the ThreadPool
	/// \note The task must not take any arguments. To add a function that does
	/// require arguments use std::bind.
	/// \param function The task to be queued
	/// \return a std::future that holds the results once completed
	template <class R>
	std::future<R> enqueue(std::function<R()> function);

private:
	/// @{
	/// Prevent default copy construction and default assignment
	ThreadPool(const ThreadPool& other);
	ThreadPool& operator=(const ThreadPool& other);
	/// @}

	/// Abstract base class for all tasks
	class TaskBase;

	/// Actual tasks, with typed return type
	template<class R>
	class Task;

	/// The worker threads
	std::vector<boost::thread> m_threads;

	/// Queued tasks waiting for an available thread
	std::queue<std::unique_ptr<TaskBase>> m_tasks;

	/// Mutex for protecting the tasks queue
	boost::mutex m_tasksMutex;

	/// Signaler for waking up threads waiting for tasks
	boost::condition_variable m_threadSignaler;

	/// True if the ThreadPool is destructing
	std::atomic<bool> m_destructing;
};

};
};

#include "SurgSim/Framework/ThreadPool-inl.h"

#endif //SURGSIM_FRAMEWORK_THREADPOOL_H



