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
#include <SurgSim/Physics/Actor.h>

namespace SurgSim
{
namespace Physics
{

class ContactCalculation;

/// Computation to determine the contacts between a list of CollisionPairs. 
/// This Computation class takes a list of actors, it will generate a list of collision pairs
/// from this list on every frame, for each CollisionPair, it uses a two dimensional table of
/// function objects (ContactCalculation) to determine how to calculate a contact between the two
/// members of each pair, if no specific function exists a default function will be used. 
/// will update the collision pairs accordingly.
/// \note HS-2013-may-24 Currently handles only RigidActor, all others  will be ignored 
class DcdCollision : public Computation
{
public:

	/// Constructor
	explicit DcdCollision(std::shared_ptr< std::vector<std::shared_ptr<Actor>>> actors);
	virtual ~DcdCollision();

	/// Gets list of all collision pairs that were considered, a pair will have
	/// either contacts or not.
	/// \return	A list of CollisionPair objects.
	const std::list<std::shared_ptr<CollisionPair>>& collisionPairs()
	{
		return m_pairs;
	}

protected:

	/// Executes the update operation, overridden from Computation.
	/// \param	dt	The time passed.
	virtual void doUpdate(double dt) override;

private:

	/// Initializes the table of ContactCalculation objects
	void populateCalculationTable();

	/// Updates the collision pairs
	void updatePairs();

	///@{
	/// Instances of factories to be used
	/// \note The ordering of the declaration here is important, CollisionPair holds
	/// 	  Contact so, when all the collision pairs get released the ContactFactory
	/// 	  needs to be available
	std::shared_ptr<SurgSim::Framework::ReuseFactory<Contact>> m_contactFactory;
	SurgSim::Framework::ReuseFactory<CollisionPair> m_pairFactory;
	///@}

	/// Table containing contact calculation, the indices indicate the type of
	/// the first pair object and the second pair object in order
	std::unique_ptr<ContactCalculation> m_contactCalculations[RIGID_SHAPE_TYPE_COUNT][RIGID_SHAPE_TYPE_COUNT];
	
	/// Global list of actors, used for creating the collision pairs
	std::shared_ptr<std::vector<std::shared_ptr<Actor>>> m_actors;

	/// List of collision pairs, recalculate every update call
	std::list<std::shared_ptr<CollisionPair>> m_pairs;
};

}; // Physics
}; // SurgSim

#endif
