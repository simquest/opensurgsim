// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_PHYSICSMANAGER_H
#define SURGSIM_PHYSICS_PHYSICSMANAGER_H

#include <boost/thread/mutex.hpp>

#include <list>
#include <memory>
#include <vector>

#include "SurgSim/Framework/ComponentManager.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Physics/PhysicsManagerState.h"


namespace SurgSim
{

namespace Collision
{
class CollisionPair;
class Representation;
}

namespace Framework
{
class Component;
}

namespace Particles
{
class Representation;
}

namespace Physics
{

class Computation;

/// PhyicsManager handles the physics and motion calculation, it uses Computations to
/// separate the algorithmic steps into smaller pieces.
/// The PhysicsManager pipeline can be configured, using the
/// addComputation/setComputaton method.
class PhysicsManager : public SurgSim::Framework::ComponentManager
{
public:

	/// Constructor
	PhysicsManager();
	virtual ~PhysicsManager();

	/// Overrides ComponentManager::getType()
	int getType() const override;

	friend class PhysicsManagerTest;

	/// Get the last PhysicsManagerState from the previous PhysicsManager update.
	/// \param [out] s pointer to an allocated PhysicsManagerState object.
	/// \warning The state contains many pointers.  The objects pointed to are not thread-safe.
	void getFinalState(SurgSim::Physics::PhysicsManagerState* s) const;

	/// Add a computation to the list of computations to perform each update
	/// \note The computations will be run in order they were added. This can't be done after initialization
	/// \param computation The Computation to add.
	void addComputation(std::shared_ptr<SurgSim::Physics::Computation> computation);

	/// Set a group of computations, the computations will be run in the order in the vector
	/// \note this can't be done after initialization
	/// \param computations The computations that you want to set on the physics manager
	void setComputations(std::vector<std::shared_ptr<Physics::Computation>> computations);

protected:
	bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component) override;

	bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component) override;

	/// Initialize the Physics Manager
	/// Derived class(es) should override this method to have a customized list of computations.
	/// \note Current default is to only use DCD contacts
	/// \sa SurgSim::Physics::Computation
	bool doInitialize() override;

	bool doStartUp() override;

	bool doUpdate(double dt) override;

	void doBeforeStop() override;

	std::vector<std::shared_ptr<Representation>> m_representations;

	std::vector<std::shared_ptr<Collision::Representation>> m_collisionRepresentations;

	std::vector<std::shared_ptr<Particles::Representation>> m_particleRepresentations;

	std::vector<std::shared_ptr<ConstraintComponent>> m_constraintComponents;

	/// A list of computations, to perform the physics update.
	std::vector<std::shared_ptr<SurgSim::Physics::Computation>> m_computations;

	/// A thread-safe copy of the last PhysicsManagerState in the previous update.
	SurgSim::Framework::LockedContainer<SurgSim::Physics::PhysicsManagerState> m_finalState;
};

/// Creates default DCD pipeline, this currently does basic DCD without regard to CCD
/// \param copyState if true the physics manager will maintain a copy of the Physics manager state for each computation
std::vector<std::shared_ptr<Physics::Computation>> createDcdPipeline(bool copyState = false);

/// Create default CCD pipeline, this currently does basic CCD without regard to DCD
/// \param copyState if true the physics manager will maintain a copy of the Physics manager state for each computation
std::vector<std::shared_ptr<Physics::Computation>> createCcdPipeline(bool copyState = false);
}; // namespace Physics
}; // namespace SurgSim

#endif
