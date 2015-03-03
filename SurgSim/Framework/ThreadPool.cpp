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


namespace SurgSim
{
namespace Framework
{

ThreadPool::ThreadPool(size_t numThreads) :
	m_destructing(false)
{
	auto threadLoop = [this] ()
	{
		std::unique_ptr<TaskBase> task;
		while (!m_destructing)
		{
			{
				boost::unique_lock<boost::mutex> lock(m_tasksMutex);
				if (m_tasks.empty())
				{
					m_threadSignaler.wait(lock);
				}

				if (!m_tasks.empty())
				{
					task = std::move(m_tasks.front());
					m_tasks.pop();
				}
			}
			if (task)
			{
				task->execute();
			}
			task.release();
		}
	};
	for (size_t i = 0; i < numThreads; i++)
	{
		m_threads.emplace_back(threadLoop);
	}
}

ThreadPool::~ThreadPool()
{
	m_destructing = true;
	m_threadSignaler.notify_all();
	for (auto& thread : m_threads)
	{
		thread.join();
	}
}

};
};
