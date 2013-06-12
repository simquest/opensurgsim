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

#ifndef SURGSIM_FRAMEWORK_RUNTIME_H
#define SURGSIM_FRAMEWORK_RUNTIME_H

#include <vector>
#include <memory>
#include <string>
#include <map>

#include <boost/thread/mutex.hpp>
#include <boost/lockfree/queue.hpp>

namespace SurgSim
{
namespace Framework
{

class ApplicationData;
class ComponentManager;
class Component;
class Logger;
class Scene;
class SceneElement;

/// This class contains all the information about the runtime environment of
/// the simulation, all the running threads, the state, while it is de facto a
/// singleton it should be passed around if needed. Needs to be created as a
/// shared object.
class Runtime : public std::enable_shared_from_this<Runtime>
{
public:

	/// Default constructor.
	Runtime();

	/// Constructor, initializes the search path with paths from the given file.
	/// \param	configFilePath 	Full pathname of the configuration file that contains paths,
	///							one per line, to search for application data. If no config file
	/// 						is given "." will be used as default path.
	explicit Runtime(const std::string& configFilePath);

	/// Destructor.
	~Runtime();

	/// Add a worker thread, this should probably only be possible if the system is not running
	void addManager(std::shared_ptr<ComponentManager> thread);

	/// Sets the scene currently only works before the runtime has started the threads
	/// \param	scene	The scene.
	void setScene(std::shared_ptr<Scene> scene);

	/// Adds a scene element
	/// \param	sceneElement	The scene element.
	/// \return	true if it succeeds, false if it fails.
	bool addSceneElement(std::shared_ptr<SceneElement> sceneElement);

	/// Start all the threads and block until one of them quits
	bool execute();

	/// Start all the threads non returns after the startup as succeeded
	/// \return	true if it succeeds, false if it fails.
	bool start();

	/// Stops the simulation.
	/// The call will wait for all the threads to finish, except for any threads that have been detached.
	/// \return	true if it succeeds, false if it fails.
	bool stop();

	/// Query if this object is running.
	/// \return	true if running, false if not.
	bool isRunning();

	/// Query if this object is paused.
	/// \return	true if paused, false if not.
	bool isPaused();

	/// Gets a shared pointer to the runtime.
	/// \return	The shared pointer.
	std::shared_ptr<Runtime> getSharedPtr();

	/// Gets a logger. If the logger does not exist yet it will be created with
	/// the given name and a default output. Cleaning up the logger will be the
	/// responsibility of this class, keep the logger reference
	/// \todo This should be moved into a future LogFactory class HS-2012-feb-05
	/// \param	loggerName	Name of the logger.
	/// \return	The logger with the name loggerName.
	std::shared_ptr<Logger> getLogger(const std::string& loggerName);

	/// Gets application data for the runtime.
	/// \return	The application data.
	std::shared_ptr<const ApplicationData> getApplicationData() const;

	void addComponent(const std::shared_ptr<Component>& component);

	void removeComponent(const std::shared_ptr<Component>& component);


private:

	/// Preprocess scene elements. This is called during the startup sequence
	/// and installs all the Components of the SceneElements in the worker threads
	void preprocessSceneElements();

	/// Adds the components.
	/// \param	components	The components.
	/// \return	true if it succeeds, false if it fails.
	void addComponents(const std::vector<std::shared_ptr<SurgSim::Framework::Component>>& components);

	/// Initializes the search paths.
	/// \param	configFilePath	Full pathname of the configuration file, if path is empty
	/// 						"." will be used as default path.
	void initSearchPaths(const std::string& configFilePath);
	bool m_isRunning;
	std::vector< std::shared_ptr<ComponentManager> > m_managers;
	std::shared_ptr<Scene> m_scene;
	std::shared_ptr<ApplicationData> m_applicationData;

	std::map<std::string, std::shared_ptr<Logger>> m_loggers;


	boost::mutex m_mutex;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_RUNTIME_H
