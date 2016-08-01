// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_COLLISION_CONTACTFILTER_H
#define SURGSIM_COLLISION_CONTACTFILTER_H


#include <SurgSim/Framework/Component.h>

#include <string>

namespace SurgSim
{

namespace Physics
{
class PhysicsManagerState;
}

namespace Collision
{
class CollisionPair;

/// Base class to implement a contact filter, the job of this class is to be executed by the ContactFiltering stage
/// in the PhysicsManager.
/// Filters are being executed in parallel over all CollisionPair objects that have contacts, be careful with thread
/// safety of the doFilterContacts function and all other writes to class members in your subclass
class ContactFilter : public Framework::Component
{
public:
	/// Constructor
	explicit ContactFilter(const std::string& name);

	/// Base function to filter the contacts on a collision pair this will modify the 'contacts' in the collision pair
	/// \param state the physics state
	/// \param pair the collision pair
	void filterContacts(
		const std::shared_ptr<Physics::PhysicsManagerState>& state,
		const std::shared_ptr<CollisionPair>& pair);

	/// Base function to update the internal data structures with regard to the filtering operation, this should only
	/// be called once per iteration
	/// \param dt the time passed since the last call to update
	void update(double dt);

protected:

	/// Override this with the implementation of the filter, this has to be threadsafe, it will be called in parallel
	/// with different collision pairs.
	virtual void doFilterContacts(const std::shared_ptr<Physics::PhysicsManagerState>& state,
								  const std::shared_ptr<CollisionPair>& pair) = 0;

	virtual void doUpdate(double dt) = 0;

};

}
}

#endif
