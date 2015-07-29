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

namespace YAML
{
class Node;
}

namespace SurgSim
{
namespace Framework
{

class ApplicationData;
class Barrier;
class ComponentManager;
class Component;
class Logger;
class Scene;
class SceneElement;
class ThreadPool;

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

	/// \return All the managers from the runtime
	std::vector<std::weak_ptr<ComponentManager>> getManagers() const;

	/// \return The scene to be used for this runtime. Use this for any kind of scene manipulation.
	std::shared_ptr<Scene> getScene();

	/// \returns The current scene
	std::shared_ptr<Scene> getScene() const;

	/// Adds a scene element
	/// \param	sceneElement	The scene element.
	/// \return	true if it succeeds, false if it fails.
	bool addSceneElement(std::shared_ptr<SceneElement> sceneElement);

	/// Start all the threads and block until one of them quits
	bool execute();

	/// Start all the threads non returns after the startup as succeeded
	/// \return	true if it succeeds, false if it fails.
	bool start(bool paused = false);

	/// Pause all managers, this will set all managers to synchronous execution, they will all complete
	/// their updates and then wait for step() to proceed, call resume to go back to uninterupted execution.
	/// \note HS-2013-nov-01 this is mostly to be used as a facillity for testing and debugging, the threads
	/// 	  are not executed at the correct rates against each other, this is an issue that can be resolved
	/// 	  but is not necessary right now.
	void pause();

	/// Resume from pause, causes all managers to resume normal processing
	/// \warning This function is not thread safe, if stop is called when there are threads that are not waiting,
	///          this call will hang indefinitely.
	void resume();

	/// Make all managers execute 1 update loop, afterwards they will wait for another step() call or resume()
	void step();

	/// Stops the simulation.
	/// The call will wait for all the threads to finish, except for any threads that have been detached.
	/// \warning This function is not thread safe, if stop is called when there are threads that are not waiting,
	///          this call will hang indefinitely.
	/// \return	true if it succeeds, false if it fails.
	bool stop();

	/// Query if this object is running.
	/// \return	true if running, false if not.
	bool isRunning() const;

	/// Query if this object is paused.
	/// \return	true if paused, false if not.
	bool isPaused() const;

	/// Gets application data for the runtime.
	/// \return	The application data.
	static std::shared_ptr<const ApplicationData> getApplicationData();

	/// Gets the thread pool for the runtime.
	/// \return	The thread pool.
	static std::shared_ptr<ThreadPool> getThreadPool();

	/// Adds a component.
	/// \param	component	The component.
	void addComponent(const std::shared_ptr<Component>& component);

	/// Removes the component described by component.
	/// \param	component	The component.
	void removeComponent(const std::shared_ptr<Component>& component);

	/// Loads the scene from the given file, clears all the elements in the scene, the old scene will be
	/// overwritten.
	/// \param fileName the filename of the scene to be loaded, needs to be found
	/// \throws If the file cannot be found or is an invalid YAML file
	void loadScene(const std::string& fileName);

	/// Adds the scene elements from the file to the current scene
	/// The file format should be just a list of sceneElements
	/// \code
	/// - SurgSim::Framework::BasicSceneElement:
	///	    Name: element1
	///	    IsActive: true
	///	    Components:
	///	      - MockComponent:
	///	          Name: component1
	///	          Id: 792faa40-459b-40cf-981d-560a8f2bd1801
	/// - SurgSim::Framework::BasicSceneElement:
	///	    Name: element2
	///	    IsActive: true
	///	    Components:
	///	      - MockComponent:
	///	          Name: component2
	///	          Id: 1de26315-82a7-46b2-ae38-324d25009629
	/// \endcode
	/// \param fileName the filename of the scene to be loaded, needs to be found
	/// \throws If the file cannot be found or is an invalid file
	void addSceneElements(const std::string& fileName);

	/// Loads and duplicates the scene elements from the file, the elements will not have common ids
	/// with any other cloned elements, this lets you repeatedly load a set of elements to replicate this
	/// set. The format is a list of scene elements \sa addSceneElements().
	/// \param fileName the filename of the scene to be loaded, needs to be found
	/// \throws if loading failed
	/// \return a vector of scene elements with the loaded elements
	std::vector<std::shared_ptr<SceneElement>> duplicateSceneElements(const std::string& fileName);


	/// Write out the whole scene as a file
	/// \param fileName the name of the scene-file if no path is given, uses the current path of the executable
	void saveScene(const std::string& fileName) const;


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

	/// Convert nodes to vector of elements
	/// \param fileName the original filename for error reporting
	/// \param node the node to be converted
	/// \param [out] elements the pointer for the results
	/// \return true if the conversion was successful
	bool tryConvertElements(const std::string& fileName, const YAML::Node& node,
							std::vector<std::shared_ptr<SceneElement>>* elements);

	/// Gets a shared pointer to the runtime.
	/// \return	The shared pointer.
	std::shared_ptr<Runtime> getSharedPtr();
	bool m_isRunning;
	std::vector< std::shared_ptr<ComponentManager> > m_managers;
	std::shared_ptr<Scene> m_scene;
	static std::shared_ptr<ApplicationData> m_applicationData;

	boost::mutex m_sceneHandling;

	std::shared_ptr<Barrier> m_barrier;
	bool m_isPaused;

	bool m_isStopped;
};

/// Perform a YAML load operation
/// \param fileName the filename of the scene to be loaded, needs to be found
/// \param [out] node pointer to the nodes structure to receive the newly loaded nodes
/// \return true if the loading succeeded
bool tryLoadNode(const std::string& fileName, YAML::Node* node);


}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_RUNTIME_H
