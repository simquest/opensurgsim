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

#ifndef SURGSIM_RUNTIME_H
#define SURGSIM_RUNTIME_H

#include <vector>
#include <memory>
#include <string>
#include <map>

#include <boost/thread/mutex.hpp>

namespace SurgSim
{
namespace Framework
{

class BasicThread;
class Scene;
class SceneElement;
class Component;
class Logger;

/// This class contains all the information about the runtime environment of
/// the simulation, all the running threads, the state, ... while it is de facto a
/// singleton it should be passed around if needed. Needs to be created as a
/// shared object
class Runtime : public std::enable_shared_from_this<Runtime>
{
public:

	/// Default constructor.
	Runtime();

	/// Destructor.
	~Runtime();

	/// Add a worker thread, this should probably only be possible if the system is not running
	void addWorkerThread(std::shared_ptr<BasicThread> thread);

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
	/// \param	waitForFinish	true to wait for all the threads to finish.
	/// \return	true if it succeeds, false if it fails.
	bool stop(bool waitForFinish);

	/// Query if this object is running.
	/// \return	true if running, false if not.
	bool isRunning();

	/// Pause the threads that are marked for pausing, if forceAll is set
	/// all the threads will be suspended no matter what
	bool setPause(bool val, bool forceAll = false);

	/// Query if this object is paused.
	/// \return	true if paused, false if not.
	bool isPaused();

	/// Gets the worker threads.
	/// \return	.
	std::vector< std::shared_ptr<BasicThread> > workerThreads();

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

private:

	/// Preprocess scene elements. This is called during the startup sequence
	/// and installs all the Components of the SceneElements in the worker threads
	void preprocessSceneElements();

	/// Adds the components.
	/// \param	components	The components.
	/// \return	true if it succeeds, false if it fails.
	bool addComponents(const std::vector<std::shared_ptr<SurgSim::Framework::Component>>& components);


	std::vector< std::shared_ptr<BasicThread> > m_workerThreads;
	std::shared_ptr<Scene> m_scene;
	bool m_isRunning;

	std::map<std::string, std::shared_ptr<Logger>> m_loggers;

	boost::mutex m_mutex;
};

};
};

#endif