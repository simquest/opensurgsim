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

#ifndef SURGSIM_PHYSICS_SCENE_IMPLEMENTATION_H
#define SURGSIM_PHYSICS_SCENE_IMPLEMENTATION_H

#include <memory>

namespace SurgSim 
{

namespace Physics
{

class ActorImplementation;
class Representation;
class ViewImplementation;

class SceneImplementation
{
public:
	SceneImplementation();
	virtual ~SceneImplementation();

	virtual bool addActor(std::shared_ptr<ActorImplementation> actor) = 0;
	virtual bool removeActor(std::shared_ptr<ActorImplementation> actor) = 0;

	virtual void update(double dt) = 0;

private:
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_SCENE_IMPLEMENTATION_H
