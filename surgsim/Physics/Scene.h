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

#ifndef SURGSIM_PHYSICS_SCENE_H
#define SURGSIM_PHYSICS_SCENE_H

#include <memory>
#include <string>
#include <unordered_map>

namespace SurgSim 
{

namespace Physics
{

class Actor;
class Representation;
class SceneImplementation;

class Scene
{
public:
	Scene(std::shared_ptr<SceneImplementation> implementation);
	virtual ~Scene();

	virtual bool addActor(std::shared_ptr<Actor> actor);
	virtual bool removeActor(std::shared_ptr<Actor> actor);
	virtual bool removeActor(const std::string& name);

	void update(double dt)
	{
		doBeforeUpdate(dt);
		doUpdate(dt);
		doAfterUpdate(dt);
	}

private:
	virtual void doBeforeUpdate(double dt);
	virtual void doUpdate(double dt);
	virtual void doAfterUpdate(double dt);

	std::unordered_map<std::string, std::shared_ptr<Actor>> m_actors;
	std::shared_ptr<SceneImplementation> m_implementation;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_SCENE_H
