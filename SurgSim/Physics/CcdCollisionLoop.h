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

namespace Framework
{
class Logger;
}

namespace Collision
{
class CollisionPair;
struct Contact;
}
namespace Physics
{

class CcdCollision;
class UpdateCcdData;
class ContactConstraintGeneration;
class BuildMlcp;
class SolveMlcp;
class PushResults;
class PhysicsManager;

class CcdCollisionLoop : public Computation
{
public:

	/// Constructor
	explicit CcdCollisionLoop(bool copyState);

	/// Destructor
	~CcdCollisionLoop();

	SURGSIM_CLASSNAME(SurgSim::Physics::CcdCollisionLoop);

	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt,
			const std::shared_ptr<PhysicsManagerState>& state) override;

	///@{
	/// Test access
	friend class CcdCollisionLoopTest_FilterContacts_Test;
	friend class CcdCollisionLoopTest_FilterContactsWithEpsilon_Test;
	///@}

	PhysicsManager* manager;

private:
	///@{
	/// Computations
	std::unique_ptr<CcdCollision> m_ccdCollision;
	std::unique_ptr<UpdateCcdData> m_updateCcdData;
	std::unique_ptr<ContactConstraintGeneration> m_constraintGeneration;
	std::unique_ptr<BuildMlcp> m_buildMlcp;
	std::unique_ptr<SolveMlcp> m_solveMlcp;
	std::unique_ptr<PushResults> m_pushResults;
	///@}

	size_t m_maxIterations; ///< maximum number of iterations to run

	/// epsilon as a fraction of dt, i.e. if this is 100, the epsilon will be dt/100
	/// during the iteration epsilon will be scaled to remain dt/100 as it pertains to the ever shrinking interval
	/// that is the iterations intervall
	double m_epsilonFactor;

	/// Takes all the contacts from ccdPairs, finds the first contact wrt contact time and removes all contacts
	/// with contact time greater than the first contact time + epsilon
	/// \param ccdPairs the list of pairs that should be checked for contacts
	/// \param epsilon the epsilon to be added to the first toi for filtering
	/// \param [out] currentToi the earliest contact time found in ccdPairs + epsilon
	/// \return true if there were any contacts found in ccdPairs
	bool filterContacts(const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs,
						double epsilon,
						double* currentToi);

	void backupContacts(const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs,
						std::vector<std::list<std::shared_ptr<Collision::Contact>>>* oldContacts);
	void restoreContacts(const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs,
						 std::vector<std::list<std::shared_ptr<Collision::Contact>>>* oldContacts);
	void printContacts(const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs);

	/// remove all the contacts from ccdPairs
	/// \param ccdPairs list of pairs for removal
	void clearContacts(std::vector<std::shared_ptr<Collision::CollisionPair>> ccdPairs);

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

}
}

#endif
