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

#include "SurgSim/Physics/PhysicsManager.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/CcdCollision.h"
#include "SurgSim/Physics/CcdCollisionLoop.h"
#include "SurgSim/Physics/ClearCollisions.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/ContactConstraintGeneration.h"
#include "SurgSim/Physics/DcdCollision.h"
#include "SurgSim/Physics/FreeMotion.h"
#include "SurgSim/Physics/ParticleCollisionResponse.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PostUpdate.h"
#include "SurgSim/Physics/PreUpdate.h"
#include "SurgSim/Physics/PrepareCollisionPairs.h"
#include "SurgSim/Physics/PublishCollisions.h"
#include "SurgSim/Physics/PushResults.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Physics/UpdateCollisionData.h"
#include "SurgSim/Physics/UpdateCollisionRepresentations.h"
#include "SurgSim/Physics/UpdateDcdData.h"

namespace SurgSim
{
namespace Physics
{

PhysicsManager::PhysicsManager() :
	ComponentManager("Physics Manager")
{
	setRate(1000.0);
}

PhysicsManager::~PhysicsManager()
{

}

int PhysicsManager::getType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

bool PhysicsManager::doInitialize()
{
	bool copyState = false;
	addComputation(std::make_shared<PreUpdate>(copyState));
	addComputation(std::make_shared<FreeMotion>(copyState));
	addComputation(std::make_shared<UpdateCollisionData>(copyState));
	addComputation(std::make_shared<PrepareCollisionPairs>(copyState));
	addComputation(std::make_shared<CcdCollisionLoop>(copyState));
	addComputation(std::make_shared<UpdateDcdData>(copyState));
	addComputation(std::make_shared<DcdCollision>(copyState));
	addComputation(std::make_shared<ContactConstraintGeneration>(copyState));
	addComputation(std::make_shared<BuildMlcp>(copyState));
	addComputation(std::make_shared<SolveMlcp>(copyState));
	addComputation(std::make_shared<PushResults>(copyState));
	addComputation(std::make_shared<ParticleCollisionResponse>(copyState));
	addComputation(std::make_shared<UpdateCollisionRepresentations>(copyState));
	addComputation(std::make_shared<PostUpdate>(copyState));

	return true;
}

void PhysicsManager::addComputation(std::shared_ptr<SurgSim::Physics::Computation> computation)
{
	m_computations.push_back(computation);
}

bool PhysicsManager::doStartUp()
{
	return true;
}

void PhysicsManager::getFinalState(SurgSim::Physics::PhysicsManagerState* s) const
{
	m_finalState.get(s);
}

bool PhysicsManager::executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	std::shared_ptr<Representation> representation = tryAddComponent(component, &m_representations);
	std::shared_ptr<Collision::Representation> collisionRep = tryAddComponent(component, &m_collisionRepresentations);
	std::shared_ptr<Particles::Representation> particles = tryAddComponent(component, &m_particleRepresentations);
	std::shared_ptr<ConstraintComponent> constraintComponent = tryAddComponent(component, &m_constraintComponents);

	return representation != nullptr || collisionRep != nullptr || particles != nullptr ||
		   constraintComponent != nullptr;
}

bool PhysicsManager::executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
{
	return tryRemoveComponent(component, &m_representations) ||
		   tryRemoveComponent(component, &m_collisionRepresentations) ||
		   tryRemoveComponent(component, &m_constraintComponents) ||
		   tryRemoveComponent(component, &m_particleRepresentations);
}

bool PhysicsManager::doUpdate(double dt)
{
	// Add all components that came in before the last update
	processComponents();

	processBehaviors(dt);

	auto state = std::make_shared<PhysicsManagerState>();
	std::list<std::shared_ptr<PhysicsManagerState>> stateList(1, state);
	state->setRepresentations(m_representations);
	state->setCollisionRepresentations(m_collisionRepresentations);
	state->setParticleRepresentations(m_particleRepresentations);
	state->setConstraintComponents(m_constraintComponents);

	for (const auto& computation : m_computations)
	{
		stateList.push_back(computation->update(dt, stateList.back()));
	}

	if (m_logger->getThreshold() <= SURGSIM_LOG_LEVEL(DEBUG))
	{
		if (m_computations.front()->getTimer().isBufferFull())
		{
			double totalTime = 0.0;
			for (const auto& computation : m_computations)
			{
				totalTime += computation->getTimer().getAverageFramePeriod();
			}
			const size_t newFrames = static_cast<size_t>(10.0 / std::max(totalTime, dt));
			for (const auto& computation : m_computations)
			{
				auto& timer = computation->getTimer();
				const double period = timer.getAverageFramePeriod();
				SURGSIM_LOG_DEBUG(m_logger)
						<< std::fixed << std::setprecision(0)
						<< computation->getClassName() << " \taverage duration " << 1e6 * period << " us (max "
						<< 1e6 * timer.getMaxFramePeriod() << " us), " << 100.0 * period / totalTime << "% of Physics.";
				timer.setMaxNumberOfFrames(newFrames);
				timer.start();
			}
		}
	}

	m_finalState.set(*(stateList.back()));

	return true;
}

void PhysicsManager::doBeforeStop()
{
	// Empty the physics manager state
	m_finalState.set(PhysicsManagerState());

	// Give all known components a chance to untangle themselves
	retireComponents(m_representations);
	retireComponents(m_particleRepresentations);
	retireComponents(m_collisionRepresentations);
	retireComponents(m_constraintComponents);

	// Call up the class hierarchy
	ComponentManager::doBeforeStop();
}

}; // Physics
}; // SurgSim
