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

#include "SurgSim/Physics/CcdCollisionLoop.h"

#include "SurgSim/Physics/CcdCollision.h"
#include "SurgSim/Physics/UpdateCcdData.h"
#include "SurgSim/Physics/ContactConstraintGeneration.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Physics/PushResults.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Collision/CollisionPair.h"

#include <unordered_set>
#include <limits>

namespace SurgSim
{

namespace Physics
{

CcdCollisionLoop::CcdCollisionLoop(bool copyState) :
	Computation(copyState),
	m_updateCcdData(new UpdateCcdData(copyState)),
	m_ccdCollision(new CcdCollision(copyState)),
	m_constraintGeneration(new ContactConstraintGeneration(copyState)),
	m_buildMlcp(new BuildMlcp(copyState)),
	m_solveMlcp(new SolveMlcp(copyState)),
	m_pushResults(new PushResults(copyState)),
	m_maxIterations(20),
	m_epsilonFactor(1000)
{
}

CcdCollisionLoop::~CcdCollisionLoop()
{

}

std::shared_ptr<SurgSim::Physics::PhysicsManagerState> CcdCollisionLoop::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	auto lastState = state;
	size_t iterations = m_maxIterations;

	auto collisionPairs = state->getCollisionPairs();
	std::vector<std::shared_ptr<Collision::CollisionPair>> ccdPairs;
	ccdPairs.reserve(collisionPairs.size());

	std::unordered_set<Collision::Representation*> representations;

	std::copy_if(collisionPairs.cbegin(), collisionPairs.cend(), std::back_inserter(ccdPairs),
				 [&representations](const std::shared_ptr<Collision::CollisionPair>& p)
	{
		if (p->getType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS)
		{
			representations.insert(p->getFirst().get());
			representations.insert(p->getSecond().get());
			return true;
		}
		return false;
	});

	// localDt is the time left on dt as we step through the CCD, this gets reduced as contacts are found
	double localDt = dt;
	double toi = 0.0;
	double localToi = 0.0;

	std::cout << "--------Iteration Start " << std::endl;

	size_t totalContacts = 0;

	while (--iterations > 0)
	{

		localDt = dt * (1 - toi);
		double epsilon = 1.0 / ((1 - toi) * m_epsilonFactor);

		// #todo ??? backup Possible contacts from DCD
		lastState = m_updateCcdData->update(localToi, lastState); // state interpolation is triggered in here
		lastState = m_ccdCollision->update(localDt, lastState);

		std::cout << ccdPairs[0]->getContacts().size() << std::endl;

		// Find the first impact and filter all contacts beyond a given epsilon
		if (!filterContacts(ccdPairs, epsilon, &localToi))
		{
			break;
		}

		lastState = m_constraintGeneration->update(localDt, lastState);
		lastState = m_buildMlcp->update(localDt, lastState);
		lastState = m_solveMlcp->update(localDt, lastState);
		lastState = m_pushResults->update(localDt, lastState);

		// set toi to the correct value as a percentage of dt
		toi += (1.0 - toi) * localToi;

		if (toi > 1.0)
		{
			break;
		}
	}

	if (iterations == 0)
	{
		std::cout << "----- Maxed out iterations ... " << std::endl;
	}
	else if (iterations < m_maxIterations - 1)
	{
		std::cout << "----- Resolved after " << m_maxIterations - iterations << std::endl;
	}

	return lastState;
}

bool CcdCollisionLoop::filterContacts(
	const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs,
	double epsilon,
	double* currentToi)
{
	SURGSIM_ASSERT(currentToi != nullptr);

	double toi = std::numeric_limits<double>::max();

	// Find earliest time of impact
	for (const auto& pair : ccdPairs)
	{
		std::for_each(pair->getContacts().begin(),
					  pair->getContacts().end(), [&toi, currentToi](const std::shared_ptr<Collision::Contact>& contact)
		{
			if (contact->time > *currentToi)
			{
				toi = std::min<double>(toi, contact->time);
			}
		});
	}

	// Did not find any contacts return false
	if (!(toi < std::numeric_limits<double>::max()))
	{
		return false;
	}

	toi += epsilon;

	for (const auto& pair : ccdPairs)
	{
		pair->getContacts().remove_if([toi](const std::shared_ptr<Collision::Contact>& contact)
		{
			return contact->time > toi;
		});
	}

	*currentToi = toi;

	return true;
}

}
}