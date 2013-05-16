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


#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#include <SurgSim/Framework/Runtime.h>

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Barrier.h>
#include <SurgSim/Framework/ComponentManager.h>
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Scene.h>

namespace SurgSim
{
namespace Framework
{


Runtime::Runtime() :
	m_isRunning(false),
	m_scene(new Scene())
{
	initSearchPaths("");
}

Runtime::Runtime(const std::string& configFilePath) :
	m_isRunning(false),
	m_scene(new Scene())
{
	initSearchPaths(configFilePath);
}

Runtime::~Runtime()
{
}

void Runtime::addManager(std::shared_ptr<ComponentManager> manager)
{
	if (manager != nullptr)
	{
		manager->setRuntime(getSharedPtr());
		m_managers.push_back(manager);
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

	result = sceneElement->initialize();

	if (result)
	{
		result = addComponents(sceneElement->getComponents());
	}

	if (result)
	{
		result = sceneElement->wakeUp();
	}

	return result;
}

bool Runtime::addComponents(const std::vector<std::shared_ptr<Component>>& components)
{
	auto componentsEnd = components.cend();
	bool result = true;
	for (auto it = components.cbegin(); it != componentsEnd; ++it)
	{
		for (auto manager = m_managers.begin(); manager != m_managers.end(); ++manager)
		{
			result = (*manager)->addComponent(*it) && result;
		}
	}
	return result;
}

bool Runtime::execute()
{
	bool doExit = false;
	if (start())
	{
		auto it = m_managers.cbegin();
		while (!doExit)
		{
			// Watchdog, shut down all managers if one manager is done
			boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
			for (it = m_managers.cbegin(); it != m_managers.cend(); ++it)
			{
				if (!(*it)->isRunning())
				{
					doExit = true;
					break;
				}
			}
		}
		stop();
	}
	return true;
}

bool Runtime::start()
{
	std::vector<std::shared_ptr<ComponentManager>>::iterator it;
	std::shared_ptr<Barrier> barrier(new Barrier(m_managers.size()+1));
	for (it = m_managers.begin(); it != m_managers.end(); ++it)
	{
		(*it)->start(barrier);
	}

	// Wait for all the managers to initialize
	barrier->wait(true);
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "All managers doInit() succeeded";


	// Wait for all the managers to startup
	barrier->wait(true);

	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "All managers doStartup() succeeded";

	m_isRunning = true;
	// Now add all the scenelements
	preprocessSceneElements();
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "Scene is initialized";

	// All managers are waiting for this
	barrier->wait(true);
	SURGSIM_LOG_INFO(Logger::getDefaultLogger()) << "All managers updating";

	return true;
}

bool Runtime::stop()
{
	std::vector<std::shared_ptr<ComponentManager>>::iterator it;
	for (it = m_managers.begin(); it != m_managers.end(); ++it)
	{
		(*it)->stop();
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
		it->second->setRuntime(getSharedPtr());
		it->second->initialize();
		std::vector<std::shared_ptr<Component>> elementComponents =  it->second->getComponents();
		newComponents.insert(newComponents.end(), elementComponents.begin(), elementComponents.end());
	}

	addComponents(newComponents);

	for (auto it = sceneElements.cbegin(); it != sceneElements.cend(); ++it)
	{
		it->second->wakeUp();
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
	catch (const std::exception& e)
	{
		SURGSIM_FAILURE() << "Runtime was not created as a shared_ptr";
	}
	return result;
}

void Runtime::initSearchPaths(const std::string& configFilePath)
{
	if (configFilePath == "")
	{
		std::vector<std::string> paths;
		paths.push_back(".");
		m_applicationData = std::make_shared<ApplicationData>(paths);
	}
	else
	{
		m_applicationData = std::make_shared<ApplicationData>(configFilePath);
	}
}

std::shared_ptr<const ApplicationData> Runtime::getApplicationData() const
{
	return m_applicationData;
}

}; // namespace Framework
}; // namespace SurgSim
