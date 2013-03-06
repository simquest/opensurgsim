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

#include <memory>
#include <string>

#include "BasicThread.h"
#include "SceneElement.h"

namespace SurgSim
{
namespace Framework
{

class Runtime;

/// Scene. Basic Container for SceneElements
class Scene
{
public:
	Scene() {};
	~Scene() {};

	/// Adds a scene element to 'element'.
	/// \param	element	The element.
	/// \return	true if it succeeds, false if it fails.
	bool addSceneElement(std::shared_ptr<SceneElement> element);

	/// Gets scene element with a given name
	/// \param	name	The name.
	/// \return	The scene element or nullptr if the element cannot be found.
	std::shared_ptr<SceneElement> getSceneElement(const std::string& name) const;

	/// Gets all the scene elements in the scene.
	/// \return	The scene elements.
	const std::map< std::string,std::shared_ptr<SceneElement> >& getSceneElements() const;

	/// Sets the runtime.
	/// \param	runtime	The runtime for this scene.
	void setRuntime(std::shared_ptr<Runtime> runtime);

private:
	std::weak_ptr<Runtime> m_runtime;

	std::map< std::string,std::shared_ptr<SceneElement> > m_elements;
};

}
}
#endif
