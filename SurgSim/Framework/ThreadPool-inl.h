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

#ifndef SURGSIM_FRAMEWORK_THREADPOOL_INL_H
#define SURGSIM_FRAMEWORK_THREADPOOL_INL_H


namespace SurgSim
{
namespace Framework
{

class ThreadPool::TaskBase
{
public:
	virtual void execute() = 0;
};

template<class R>
class ThreadPool::Task : public ThreadPool::TaskBase
{
public:
	explicit Task(std::function<R()> function) :
		m_task(function)
	{
	}

	void execute() override
	{
		m_task();
	}

	std::future<R> getFuture()
	{
		return  m_task.get_future();
	}

private:
	std::packaged_task<R()> m_task;
};

template <class R>
std::future<R> ThreadPool::enqueue(std::function<R()> function)
{
	std::unique_ptr<Task<R>> task = std::unique_ptr<Task<R>>(new Task<R>(function));
	std::future<R> future = task->getFuture();
	{
		boost::unique_lock<boost::mutex> lock(m_tasksMutex);
		m_tasks.push(std::move(task));
	}
	m_threadSignaler.notify_one();
	return future;
}

};
};

#endif //SURGSIM_FRAMEWORK_THREADPOOL_INL_H


