// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest LLC.
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


#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#include <Surgsim/Framework/Runtime.h>
#include <Surgsim/Framework/BasicThread.h>
#include <Surgsim/Framework/Scene.h>
#include <Surgsim/Framework/Component.h>
#include <Surgsim/Framework/Barrier.h>
#include <Surgsim/Framework/Log.h>

namespace SurgSim
{
namespace Framework
{


Runtime::Runtime() :
	m_isRunning(false),
	m_scene(new Scene())
{
}

Runtime::~Runtime()
{

}

void Runtime::addWorkerThread(std::shared_ptr<BasicThread> thread)
{
	if (thread != nullptr)
	{
		thread->setRuntime(getSharedPtr());
		m_workerThreads.push_back(thread);
	}
}

void SurgSim::Framework::Runtime::setScene(std::shared_ptr<Scene> scene)
{
	// Workers need to be initialized to do this
	SURGSIM_ASSERT(! m_isRunning) << "Cannot set the Scene in the Runtime once it is running";
	m_scene = scene;
	scene->setRuntime(getSharedPtr());
}

bool Runtime::addSceneElement(std::shared_ptr<SceneElement> sceneElement)
{
	// If we add a single scene element before the simulation is running
	// it will be handled by the scene initialization
	if (! m_isRunning)
	{
		return false;
	}

	bool result = false;

	result = sceneElement->doInit();

	if (result)
	{
		result = addComponents(sceneElement->getComponents());
	}

	if (result)
	{
		result = sceneElement->doWakeUp();
	}

	return result;
}

bool Runtime::addComponents(const std::vector<std::shared_ptr<Component>>& components)
{
	auto componentsEnd = components.cend();
	bool result = true;
	for (auto it = components.cbegin(); it != componentsEnd; ++it)
	{
		for (auto thread = m_workerThreads.begin(); thread != m_workerThreads.end(); ++thread)
		{
			result = (*thread)->addComponent(*it) && result;
		}
	}
	return result;
}

bool Runtime::execute()
{
	bool doExit = false;
	if (start())
	{
		auto it = m_workerThreads.cbegin();
		while (!doExit)
		{
			// Watchdog, shut down all threads if one thread is done
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			for (it = m_workerThreads.cbegin(); it != m_workerThreads.cend(); ++it)
			{
				if (!(*it)->isRunning())
				{
					doExit = true;
					break;
				}
			}
		}
		stop(true);
	}
	return true;
}

bool Runtime::start()
{
	std::vector<std::shared_ptr<BasicThread>>::iterator it;
	std::shared_ptr<Barrier> barrier(new Barrier(m_workerThreads.size()+1));
	for (it = m_workerThreads.begin(); it != m_workerThreads.end(); ++it)
	{
		(*it)->doRun(barrier);
	}

	// Wait for all the threads to initialize
	barrier->wait(true);
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "All threads doInit() suceeded";


	// Wait for all the threads to startup
	barrier->wait(true);

	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "All threads doStartup() suceeded";

	m_isRunning = true;
	// Now add all the scenelements
	preprocessSceneElements();
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "Scene is initialized";

	// All threads are waiting for this
	barrier->wait(true);
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "All threads updating";

	return true;
}

bool Runtime::stop(bool waitForFinish)
{
	std::vector<std::shared_ptr<BasicThread>>::iterator it;
	for (it = m_workerThreads.begin(); it != m_workerThreads.end(); ++it)
	{
		(*it)->doStop(waitForFinish);
	}
	return true;
}

void Runtime::preprocessSceneElements()
{
	// Collect all the Components
	std::vector<std::shared_ptr<Component>> newComponents;
	auto sceneElements = m_scene->getSceneElements();
	for (auto it = sceneElements.begin(); it != sceneElements.end(); ++it)
	{
		it->second->doInit();
		std::vector<std::shared_ptr<Component>> elementComponents =  it->second->getComponents();
		newComponents.insert(newComponents.end(), elementComponents.begin(), elementComponents.end());
	}

	addComponents(newComponents);

	for (auto it = sceneElements.cbegin(); it != sceneElements.cend(); ++it)
	{
		it->second->doWakeUp();
	}
}

std::shared_ptr<Logger> Runtime::getLogger(const std::string& loggerName)
{
	std::shared_ptr<Logger> logger = m_loggers[loggerName];
	if (logger == nullptr)
	{
		boost::mutex::scoped_lock lock(m_mutex);
		// Check again in the critical section
		logger = m_loggers[loggerName];
		if (logger == nullptr)
		{
			logger = std::make_shared<Logger>(loggerName, std::make_shared<StreamOutput>(std::cout));
			m_loggers[loggerName] = logger;
		}
	}
	return logger;
}

std::shared_ptr<Runtime> Runtime::getSharedPtr()
{
	std::shared_ptr<Runtime> result;
	try
	{
		result = shared_from_this();
	}
	catch (std::exception e)
	{
		SURGSIM_FAILURE() << "Runtime was not created as a shared_ptr";
	}
	return result;
}

}; // namespace Framework
}; // namespace SurgSim
