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

#ifndef SURGSIM_FRAMEWORK_SCENE_H
#define SURGSIM_FRAMEWORK_SCENE_H

#include <vector>
#include <memory>
#include <string>
#include <boost/thread/mutex.hpp>

#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/DataStructures/Groups.h"

namespace YAML
{
class Node;
}

namespace SurgSim
{
namespace Framework
{

class Runtime;

/// Scene. Basic Container for SceneElements
class Scene : public std::enable_shared_from_this<Scene>
{
public:

	typedef SurgSim::DataStructures::Groups<std::string, std::shared_ptr<SceneElement>> GroupsType;

	/// Constructor.
	/// \param runtime The runtime to be used.
	explicit Scene(std::weak_ptr<Runtime> runtime);

	/// Destructor
	~Scene();

	/// Adds a scene element to the Scene, the SceneElement will have its initialize() function called.
	/// \param	element	The element.
	void addSceneElement(std::shared_ptr<SceneElement> element);

	/// Invokes addSceneElement() for each element in the list.
	/// \param elements the list of elements to be added.
	void addSceneElements(std::vector<std::shared_ptr<SceneElement>> elements);

	/// Gets all the scene elements in the scene.
	/// \return	The scene elements.
	const std::vector<std::shared_ptr<SceneElement>>& getSceneElements() const;

	/// Retrieve a SceneElement for this scene with the given name.
	/// \return A SceneElement with given name; Empty share_ptr<> will be returned if no such SceneElement found.
	const std::shared_ptr<SceneElement> getSceneElement(const std::string& name) const;

	/// Gets the runtime.
	/// \return runtime The runtime for this scene.
	std::shared_ptr<Runtime> getRuntime();

	/// Convert to a YAML::Node
	/// \return A node with all the public data of this instance
	YAML::Node encode() const;

	/// Pull data from a YAML::Node.
	/// \param node the node to decode.
	/// \return true if the decoding succeeded and the node was formatted correctly, false otherwise
	bool decode(const YAML::Node& node);

	/// \return the groups of the scene
	std::shared_ptr<GroupsType> getGroups();

private:

	/// \return The shared pointer.
	std::shared_ptr<Scene> getSharedPtr();

	std::weak_ptr<Runtime> m_runtime;

	std::vector<std::shared_ptr<SceneElement>> m_elements;

	// Used in a const function, need to declare mutable
	mutable boost::mutex m_sceneElementsMutex;

	std::shared_ptr<GroupsType> m_groups;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_SCENE_H
