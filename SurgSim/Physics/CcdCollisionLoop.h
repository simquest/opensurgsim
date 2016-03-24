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

#ifndef SURGSIM_PHYSICS_CCDCOLLISIONLOOP_H
#define SURGSIM_PHYSICS_CCDCOLLISIONLOOP_H

#include "SurgSim/Physics/ComputationGroup.h"

namespace SurgSim
{

namespace Collision
{
class CollisionPair;
}
namespace Physics
{

class CcdCollision;
class UpdateCcdData;
class ContactConstraintGeneration;
class BuildMlcp;
class SolveMlcp;
class PushResults;

class CcdCollisionLoop : public Computation
{
public:
	friend class CcdCollisionLoopTest_FilterContacts_Test;

	/// Constructor
	explicit CcdCollisionLoop(bool copyState);

	/// Destructor
	~CcdCollisionLoop();

	SURGSIM_CLASSNAME(SurgSim::Physics::CcdCollisionLoop);

	virtual std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt,
			const std::shared_ptr<PhysicsManagerState>& state) override;

private:
	///@{
	/// Computations
	std::unique_ptr<CcdCollision> m_ccdCollision;
	std::unique_ptr<UpdateCcdData> m_updateCcdData;
	std::unique_ptr<ContactConstraintGeneration> m_constraintGeneration;
	std::unique_ptr<BuildMlcp> m_buildMlcp;
	std::unique_ptr<SolveMlcp> m_solveMlcp;
	std::unique_ptr<PushResults> m_pushResults;

	size_t m_maxIterations;
	double m_epsilonFactor;

	bool filterContacts(const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs,
						double epsilon,
						double* currentToi);
};

}
}

#endif
