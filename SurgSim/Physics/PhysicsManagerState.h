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

#ifndef SURGSIM_PHYSICS_PHYSICSMANAGERSTATE_H
#define SURGSIM_PHYSICS_PHYSICSMANAGERSTATE_H

#include <memory>
#include <vector>

namespace SurgSim
{
namespace Physics
{

class Actor;
class CollisionPair;

class PhysicsManagerState
{
public:

	/// Constructor
	PhysicsManagerState() {}
	~PhysicsManagerState() {}

	const std::vector<std::shared_ptr<Actor>>& getActors() const
	{ return m_actors; }
	void setActors(const std::vector<std::shared_ptr<Actor>>& val)
	{ m_actors = val; }

	const std::vector<std::shared_ptr<CollisionPair>>& getCollisionPairs() const
	{ return m_collisionPairs; }
	void setCollisionPairs(std::vector<std::shared_ptr<CollisionPair>> val)
	{ m_collisionPairs = val; }

private:

	std::vector<std::shared_ptr<Actor>> m_actors;
	std::vector<std::shared_ptr<CollisionPair>> m_collisionPairs;

};

}; // Physics
}; // SurgSim

#endif
