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

#ifndef SURGSIM_GRAPHICS_SCENE_H
#define SURGSIM_GRAPHICS_SCENE_H

#include <memory>
#include <string>
#include <unordered_map>

namespace SurgSim 
{

namespace Graphics
{

class Actor;
class Representation;
class SceneImplementation;

/// Graphics scene, which is a collection of graphics actors to provide a visualization of the virtual scene.
class Scene
{
public:
	/// Constructor
	/// \param	implementation	Implementation of the scene, which providing the low-level functionality.
	Scene(std::shared_ptr<SceneImplementation> implementation);
	/// Destructor
	virtual ~Scene();

	/// Adds an actor to this scene.
	bool addActor(std::shared_ptr<Actor> actor);
	/// Removes an actor from this scene.
	bool removeActor(std::shared_ptr<Actor> actor);
	/// Removes an actor from this scene by name.
	bool removeActor(const std::string& name);

	/// Updates the scene for a single timestep.
	/// The doUpdate(double) method provides the functionality to update the scene.
	/// \param	dt	The time in seconds of the preceding timestep.
	void update(double dt)
	{
		doUpdate(dt);
	}

	/// Returns the implementation of this scene, which provides the low-level functionality.
	std::shared_ptr<SceneImplementation> getImplementation() const
	{
		return m_implementation;
	}

private:
	/// Map name to actor.
	std::unordered_map<std::string, std::shared_ptr<Actor>> m_actors;
	/// Implementation of this scene, providing low-level functionality.
	std::shared_ptr<SceneImplementation> m_implementation;

	/// Updates the scene for a single timestep.
	/// Override this method for custom functionality.
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void doUpdate(double dt);
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SCENE_H
