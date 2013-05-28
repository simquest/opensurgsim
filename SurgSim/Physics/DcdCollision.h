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

#ifndef SURGSIM_PHYSICS_DCDCOLLISION_H
#define SURGSIM_PHYSICS_DCDCOLLISION_H

#include <memory>
#include <vector>
#include <list>


#include <SurgSim/Framework/ReuseFactory.h>

#include <SurgSim/Physics/Computation.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/Actors/Actor.h>

namespace SurgSim
{
namespace Physics
{

class ContactCalculation;

/// Computation to determine the contacts between a list of CollisionPairs
/// will update the collision pairs accordingly
/// \note HS-2013-may-24 Currently handles only RigidActor, all others  will be ignored 
class DcdCollision : public Computation
{
public:

	/// Constructor
	explicit DcdCollision(std::shared_ptr< std::vector<std::shared_ptr<Actor>>> actors);
	virtual ~DcdCollision();

	const std::list<std::shared_ptr<CollisionPair>>& collisionPairs()
	{
		return m_pairs;
	}

protected:
	void doUpdate(double dt);

private:
	void populateCollisionTable();
	void updatePairs();
	size_t m_pairCount;

	SurgSim::Framework::ReuseFactory<CollisionPair> m_pairFactory;

	std::list<std::shared_ptr<CollisionPair>> m_pairs;
	std::unique_ptr<ContactCalculation> m_contactCalculations[RIGID_SHAPE_TYPE_COUNT][RIGID_SHAPE_TYPE_COUNT];

	std::shared_ptr< std::vector<std::shared_ptr<Actor>>> m_actors;
};

}; // Physics
}; // SurgSim

#endif
