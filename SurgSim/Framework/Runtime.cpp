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

#include "SurgSim/Framework/Runtime.h"

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Barrier.h"
#include "SurgSim/Framework/ComponentManager.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace Framework
{

std::shared_ptr<ApplicationData> Runtime::m_applicationData;

Runtime::Runtime() :
	m_isRunning(false),
	m_isPaused(false),
	m_isStopped(false)
{
	initSearchPaths("");
}

Runtime::Runtime(const std::string& configFilePath) :
	m_isRunning(false),
	m_isPaused(false),
	m_isStopped(false)
{
	initSearchPaths(configFilePath);
}

Runtime::~Runtime()
{
	// Kill all threads
	stop();
}

void Runtime::addManager(std::shared_ptr<ComponentManager> manager)
{
	SURGSIM_ASSERT(! m_isRunning) << "Cannot add a manager to the runtime once it is running";

	if (manager != nullptr)
	{
		manager->setRuntime(getSharedPtr());
		m_managers.push_back(manager);
	}
}

std::shared_ptr<Scene> Runtime::getScene()
{
	if (m_scene == nullptr)
	{
		m_scene = std::make_shared<Scene>(getSharedPtr());
	}
	return m_scene;
}


bool Runtime::addSceneElement(std::shared_ptr<SceneElement> sceneElement)
{
	// Before the runtime has been started, adding components will be handled via
	// preprocessSceneElements()
	if (m_isRunning)
	{
		addComponents(sceneElement->getComponents());
	}

	return m_isRunning;
}

void Runtime::addComponents(const std::vector<std::shared_ptr<SurgSim::Framework::Component>>& components)
{
	auto componentsIt = std::begin(components);
	auto componentsEnd = std::end(components);
	for (; componentsIt != componentsEnd; ++componentsIt)
	{
		for (auto manager = std::begin(m_managers); manager != std::end(m_managers); ++manager)
		{
			if ((*componentsIt)->isInitialized())
			{
				(*manager)->enqueueAddComponent(*componentsIt);
			}
			else
			{
				SURGSIM_LOG_WARNING(Logger::getLogger("Runtime")) <<
						"Trying to add an uninitialized component.";
			}
		}
	}
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

bool Runtime::start(bool paused)
{
	auto logger = Logger::getDefaultLogger();

	SURGSIM_ASSERT(m_isStopped == false) << "This runtime has already been stopped, it cannot be started again.";

	// Gather all the Components from the currently known SceneElements to add them
	// collectively.
	// HS-2013-dec-12 This construct cause a bug as this also gathers the elements to be processed
	// any sceneelements added after this call and before initialization is finished will get lost
	preprocessSceneElements();

	m_isRunning = true;
	m_isPaused = paused;

	std::vector<std::shared_ptr<ComponentManager>>::iterator it;
	m_barrier.reset(new Barrier(m_managers.size() + 1));
	for (it = m_managers.begin(); it != m_managers.end(); ++it)
	{
		(*it)->start(m_barrier, m_isPaused);
	}

	// Wait for all the managers to initialize
	m_barrier->wait(true);
	SURGSIM_LOG_INFO(logger) << "All managers doInit() succeeded";

	// Wait for all the managers to startup
	m_barrier->wait(true);
	SURGSIM_LOG_INFO(logger) << "All managers doStartup() succeeded";

	// Wait for all the components to initialize()
	m_barrier->wait(true);
	SURGSIM_LOG_INFO(logger) << "All component initialize() succeeded";

	// Wait for all the components to wakeUp()
	m_barrier->wait(true);
	SURGSIM_LOG_INFO(logger) << "All component wakeUp() succeeded";
	SURGSIM_LOG_INFO(logger) << "Scene is initialized. All managers updating";

	return true;
}

bool Runtime::stop()
{
	if (m_isStopped == true)
	{
		return false;
	}

	if (isPaused())
	{
		resume();
	}

	m_isRunning = false;

	// Suspend updates on all threads
	std::vector<std::shared_ptr<ComponentManager>>::iterator it;
	for (it = m_managers.begin(); it != m_managers.end(); ++it)
	{
		(*it)->setIdle(true);
	}

	// Give all threads time to run through update
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	// Now stop all threads
	for (it = m_managers.begin(); it != m_managers.end(); ++it)
	{
		SurgSim::Framework::Timer timer;
		timer.start();
		(*it)->stop();
		timer.endFrame();
		SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Killing : " << (*it)->getName() << " took " << timer.getCumulativeTime() << "s";
	}

	m_isStopped = true;

	return true;
}

void Runtime::pause()
{
	m_isPaused = true;
	for (auto it = std::begin(m_managers); it != std::end(m_managers); ++it)
	{
		(*it)->setSynchronous(true);
	}
}

void Runtime::resume()
{
	if (isPaused())
	{
		m_isPaused = false;
		for (auto it = std::begin(m_managers); it != std::end(m_managers); ++it)
		{
			(*it)->setSynchronous(false);
		}
		// HS-2014-feb-21 if there are threads that are not waiting this will hang, this can happen if the above call
		// to setSynchronous was made while the thread was executing code rather than waiting.
		// #threadsafety
		m_barrier->wait(true);
	}
}

void Runtime::step()
{
	if (isPaused())
	{
		m_barrier->wait(true);
	}
}

bool Runtime::isRunning() const
{
	return m_isRunning;
}

bool Runtime::isPaused() const
{
	return m_isPaused;
}

void Runtime::preprocessSceneElements()
{
	// Collect all the Components
	std::vector<std::shared_ptr<Component>> newComponents;
	auto sceneElements = getScene()->getSceneElements();
	for (auto it = sceneElements.begin(); it != sceneElements.end(); ++it)
	{
		// Initialize should have been called by now, if the SceneElement is not initialized this means
		// initialization failed
		if ((*it)->isInitialized())
		{
			std::vector<std::shared_ptr<Component>> elementComponents = (*it)->getComponents();
			newComponents.insert(newComponents.end(), elementComponents.begin(), elementComponents.end());
		}
	}

	addComponents(newComponents);
}

std::shared_ptr<Runtime> Runtime::getSharedPtr()
{
	std::shared_ptr<Runtime> result;
	try
	{
		result = shared_from_this();
	}
	catch (const std::exception&)
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

std::shared_ptr<const ApplicationData> Runtime::getApplicationData()
{
	SURGSIM_ASSERT(nullptr != m_applicationData) <<
			"Runtime::getApplicationData() should be called after the Runtime is created.";
	return m_applicationData;
}

void Runtime::addComponent(const std::shared_ptr<Component>& component)
{
	if (m_isRunning)
	{
		for (auto it = std::begin(m_managers); it != std::end(m_managers); ++it)
		{
			(*it)->enqueueAddComponent(component);
		}
	}
}

void Runtime::removeComponent(const std::shared_ptr<Component>& component)
{
	if (m_isRunning)
	{
		for (auto it = std::begin(m_managers); it != std::end(m_managers); ++it)
		{
			(*it)->enqueueRemoveComponent(component);
		}
	}
}

void Runtime::loadScene(const std::string& fileName)
{
	YAML::Node node;
	boost::lock_guard<boost::mutex> lock(m_sceneHandling);
	if (tryLoadNode(fileName, &node))
	{
		YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::getRegistry().clear();
		m_scene = std::make_shared<Scene>(getSharedPtr());
		m_scene->decode(node);
	}
	else
	{
		SURGSIM_FAILURE() << "Loading of the Scene failed.";
	}

}

void Runtime::addSceneElements(const std::string& fileName)
{
	YAML::Node node;
	boost::lock_guard<boost::mutex> lock(m_sceneHandling);

	if (tryLoadNode(fileName, &node))
	{
		std::vector<std::shared_ptr<SceneElement>> elements;
		if (tryConvertElements(fileName, node, &elements))
		{
			getScene()->addSceneElements(elements);
		}
	}
	else
	{
		SURGSIM_FAILURE() << "Could not add scene elements from the YAML file: " << fileName;
	}
}

std::vector<std::shared_ptr<SceneElement>> Runtime::duplicateSceneElements(const std::string& fileName)
{
	YAML::Node node;
	YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::RegistryType registry;
	std::vector<std::shared_ptr<SceneElement>> result;

	boost::lock_guard<boost::mutex> lock(m_sceneHandling);

	// Use a temporary registry
	std::swap(YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::getRegistry(), registry);

	bool success = false;
	if (tryLoadNode(fileName, &node) && tryConvertElements(fileName, node, &result))
	{
		success = true;
	}

	// restore the original registry
	std::swap(YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::getRegistry(), registry);

	if (!success)
	{
		SURGSIM_FAILURE() << "Could not clone from the YAML file: " << fileName;
	}

	return result;
}

bool tryLoadNode(const std::string& fileName, YAML::Node* node)
{
	bool result = false;
	std::string path;
	SURGSIM_ASSERT(node != nullptr) << "Can't load node into nullptr.";
	if (Runtime::getApplicationData()->tryFindFile(fileName, &path))
	{
		try
		{
			*node = YAML::LoadFile(path);
			result = true;
		}
		catch (YAML::ParserException e)
		{
			SURGSIM_LOG_SEVERE(Logger::getLogger("Runtime")) << "Could not parse YAML File at " << path
					<< " due to " << e.msg << " at line " << e.mark.line << " column " << e.mark.column;
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(Logger::getLogger("Runtime")) << "Could not find file " << fileName;
	}
	return result;
}

bool Runtime::tryConvertElements(const std::string& filename, const YAML::Node& node,
								 std::vector<std::shared_ptr<SceneElement>>* elements)
{
	bool result = false;
	if (node.IsSequence())
	{
		try
		{
			*elements = node.as<std::vector<std::shared_ptr<SceneElement>>>();
			result = true;
		}
		catch (YAML::Exception e)
		{
			SURGSIM_LOG_SEVERE(Logger::getLogger("Runtime"))
					<< "File " << filename << " YAML conversion failed with exception: " + e.msg;
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(Logger::getLogger("Runtime"))
				<< "File " << filename << " not a YAML sequence, can't load scene elements.";
	}
	return result;
}

void Runtime::saveScene(const std::string& fileName) const
{
	std::ofstream out(fileName);
	if (out.good())
	{
		out << m_scene->encode();
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getLogger("Runtime"))
				<< "Failed to open " <<  fileName << ". Cannot save the scene.";
	}
}

}; // namespace Framework
}; // namespace SurgSim
