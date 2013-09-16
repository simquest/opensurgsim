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

#include <memory>
#include <vector>

#include <SurgSim/Framework/ComponentManager.h>
#include <SurgSim/Framework/Component.h>
#include <SurgSim/Framework/Log.h>


namespace SurgSim
{
namespace Framework
{
	class Logger;
	class Component;
}

namespace Collision
{
	class CollisionRepresentation;
}
namespace Physics
{

class BuildMlcp;
class ContactConstraintGeneration;
class FreeMotion;
class DcdCollision;
class PostUpdate;
class PreUpdate;
class PushResults;
class Representation;
class SolveMlcp;
using SurgSim::Collision::CollisionRepresentation;

/// PhyicsManager handles the physics and motion calculation, it uses Computations to
/// separate the algorithmic steps into smaller pieces.
class PhysicsManager : public SurgSim::Framework::ComponentManager
{
public:

	/// Constructor
	PhysicsManager();
	virtual ~PhysicsManager();

	friend class PhysicsManagerTest;

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

	std::vector<std::shared_ptr<Representation>> m_representations;

	std::vector<std::shared_ptr<CollisionRepresentation>> m_collisionRepresentations;


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
	///@}

};

}; // namespace Physics
}; // namespace SurgSim



#endif