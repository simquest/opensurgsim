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

#ifndef SURGSIM_PHYSICS_PHYSICSMANAGER_H
#define SURGSIM_PHYSICS_PHYSICSMANAGER_H

#include <boost/thread/mutex.hpp>
#include <memory>
#include <vector>

#include "SurgSim/Framework/ComponentManager.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Physics/PhysicsManagerState.h"


namespace SurgSim
{
namespace Framework
{
class Component;
}

namespace Collision
{
class CollisionPair;
class Representation;
}
namespace Physics
{

class BuildMlcp;
class ConstraintComponent;
class ContactConstraintGeneration;
class DcdCollision;
class FreeMotion;
class PostUpdate;
class PreUpdate;
class PushResults;
class Representation;
class SolveMlcp;
class UpdateCollisionRepresentations;

/// PhyicsManager handles the physics and motion calculation, it uses Computations to
/// separate the algorithmic steps into smaller pieces.
class PhysicsManager : public SurgSim::Framework::ComponentManager
{
public:

	/// Constructor
	PhysicsManager();
	virtual ~PhysicsManager();

	/// Overrides ComponentManager::getType()
	virtual int getType() const override;

	friend class PhysicsManagerTest;

	/// Get the last PhysicsManagerState from the previous PhysicsManager update.
	/// \param [out] s pointer to an allocated PhysicsManagerState object.
	/// \warning The state contains many pointers.  The objects pointed to are not thread-safe.
	void getFinalState(SurgSim::Physics::PhysicsManagerState* s) const;

	/// Add an excluded collision pair to the Physics Manager.  The pair will not participate in collisions.
	/// \param representation1 The first Collision::Representation for the pair
	/// \param representation2 The second Collision::Representation for the pair
	void addExcludedCollisionPair(std::shared_ptr<SurgSim::Collision::Representation> representation1,
								  std::shared_ptr<SurgSim::Collision::Representation> representation2);

	/// Remove an excluded collision pair to the Physics Manager.  The pair will not be excluded from collisions.
	/// \param representation1 The first Collision::Representation for the pair
	/// \param representation2 The second Collision::Representation for the pair
	void removeExcludedCollisionPair(std::shared_ptr<SurgSim::Collision::Representation> representation1,
									 std::shared_ptr<SurgSim::Collision::Representation> representation2);

	/// Set the maximum number of iterations for the MLCP solver.
	/// \param maxIterations The maximum number of iterations.
	void setMaxIterations(int maxIterations);

	/// Get the maximum number of iterations for the MLCP solver.
	/// \return The maximum number of iterations.
	int PhysicsManager::getMaxIterations() const;

	/// Set the precision of the MLCP solver.
	/// \param precision The precision.
	void setPrecision(double precision);

	/// Get the precision of the MLCP solver.
	/// \return The precision.
	double getPrecision() const;

	/// Set the contact tolerance for the MLCP solver.
	/// \param epsilon The contact tolerance.
	void setContactTolerance(double contactTolerance);

	/// Get the contact tolerance for the MLCP solver.
	/// \return The contact tolerance.
	double getContactTolerance() const;

protected:
	///@{
	/// Overridden from ComponentManager
	bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component) override;
	bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component) override;
	///@}

	///@{
	/// Overridden from BasicThread
	virtual bool doInitialize() override;
	virtual bool doStartUp() override;
	virtual bool doUpdate(double dt) override;
	///@}

	void initializeComputations(bool copyState);
private:
	/// Get an iterator to an excluded collision pair.
	/// \note Lock m_excludedCollisionPairMutex before calling
	/// \param representation1 The first Collision::Representation for the pair
	/// \param representation2 The second Collision::Representation for the pair
	/// \return If the pair is found, an iterator to the excluded collision pair; otherwise an iterator to the
	/// container's past-the-end element.
	std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>>::iterator findExcludedCollisionPair(
		std::shared_ptr<SurgSim::Collision::Representation> representation1,
		std::shared_ptr<SurgSim::Collision::Representation> representation2);

	std::vector<std::shared_ptr<Representation>> m_representations;

	std::vector<std::shared_ptr<SurgSim::Collision::Representation>> m_collisionRepresentations;

	std::vector<std::shared_ptr<ConstraintComponent>> m_constraintComponents;

	/// List of Collision::Representation pairs to be excluded from contact generation.
	std::vector<std::shared_ptr<SurgSim::Collision::CollisionPair>> m_excludedCollisionPairs;

	/// Mutex to protect m_excludedCollisionPairs from being read/written simultaneously.
	boost::mutex m_excludedCollisionPairMutex;

	///@{
	/// Steps to perform the physics update
	std::unique_ptr<PreUpdate> m_preUpdateStep;
	std::unique_ptr<FreeMotion> m_freeMotionStep;
	std::unique_ptr<DcdCollision> m_dcdCollisionStep;
	std::unique_ptr<ContactConstraintGeneration> m_constraintGenerationStep;
	std::unique_ptr<BuildMlcp> m_buildMlcpStep;
	std::unique_ptr<SolveMlcp> m_solveMlcpStep;
	std::unique_ptr<PushResults> m_pushResultsStep;
	std::unique_ptr<PostUpdate> m_postUpdateStep;
	std::unique_ptr<UpdateCollisionRepresentations> m_updateCollisionRepresentationsStep;
	///@}

	///@{
	/// Optional parameters to pass the SolveMlcp class.
	SurgSim::DataStructures::OptionalValue<double> m_precision;
	SurgSim::DataStructures::OptionalValue<double> m_contactTolerance;
	SurgSim::DataStructures::OptionalValue<unsigned int> m_maxIterations;
	///@}

	/// A thread-safe copy of the last PhysicsManagerState in the previous update.
	SurgSim::Framework::LockedContainer<SurgSim::Physics::PhysicsManagerState> m_finalState;
};

}; // namespace Physics
}; // namespace SurgSim



#endif
