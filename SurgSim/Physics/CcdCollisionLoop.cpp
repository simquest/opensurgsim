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

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/CcdCollision.h"
#include "SurgSim/Physics/CcdCollisionLoop.h"
#include "SurgSim/Physics/ContactConstraintGeneration.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/PushResults.h"
#include "SurgSim/Physics/SolveMlcp.h"
#include "SurgSim/Physics/UpdateCcdData.h"

#include <unordered_set>
#include <limits>

namespace SurgSim
{

namespace Physics
{

CcdCollisionLoop::CcdCollisionLoop(bool copyState) :
	Computation(copyState),
	m_ccdCollision(new CcdCollision(copyState)),
	m_updateCcdData(new UpdateCcdData(copyState)),
	m_constraintGeneration(new ContactConstraintGeneration(copyState)),
	m_buildMlcp(new BuildMlcp(copyState)),
	m_solveMlcp(new SolveMlcp(copyState)),
	m_pushResults(new PushResults(copyState)),
	m_maxIterations(20),
	m_epsilonFactor(100),
	m_logger(SurgSim::Framework::Logger::getLogger("Physics/CCDCollisionLoop"))
{
}

CcdCollisionLoop::~CcdCollisionLoop()
{

}

std::shared_ptr<PhysicsManagerState> CcdCollisionLoop::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	auto ccdState = state;

	auto& collisionPairs = state->getCollisionPairs();
	std::vector<std::shared_ptr<Collision::CollisionPair>> ccdPairs;
	ccdPairs.reserve(collisionPairs.size());

	std::copy_if(collisionPairs.cbegin(), collisionPairs.cend(), std::back_inserter(ccdPairs),
				 [](const std::shared_ptr<Collision::CollisionPair>& p)
	{
		return p->getType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS;
	});

	double timeOfImpact = 0.0;
	double localTimeOfImpact = 0.0;
	std::vector<std::list<std::shared_ptr<Collision::Contact>>> oldContacts;

	size_t iterations = 0;
	for (; iterations < m_maxIterations; ++iterations)
	{
		double epsilon = 1.0 / ((1 - timeOfImpact) * m_epsilonFactor);

		ccdState = m_updateCcdData->update(localTimeOfImpact, ccdState); // state interpolation is triggered in here
		ccdState = m_ccdCollision->update(dt, ccdState);

		if (m_logger->getThreshold() <= SurgSim::Framework::LOG_LEVEL_DEBUG)
		{
			printContacts(ccdPairs);
		}

		// Find the first impact and filter all contacts beyond a given epsilon
		if (!findEarliestContact(ccdPairs, &localTimeOfImpact))
		{
			break;
		}
		filterLaterContacts(&ccdPairs, epsilon, localTimeOfImpact);

		restoreContacts(&ccdPairs, &oldContacts);

		ccdState = m_constraintGeneration->update(dt, ccdState);
		ccdState = m_buildMlcp->update(dt, ccdState);
		ccdState = m_solveMlcp->update(dt, ccdState);
		ccdState = m_pushResults->update(dt, ccdState);

		backupContacts(&ccdPairs, &oldContacts);

		timeOfImpact += (1.0 - timeOfImpact) * localTimeOfImpact;
		if (timeOfImpact > 1.0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Calculated time of impact is greater " <<
										 "than the parametric upper bound of 1.0 (" <<
										 timeOfImpact << ")" << std::endl;
			break;
		}
	}

	SURGSIM_LOG_IF(iterations == m_maxIterations, m_logger, WARNING) <<
			"Maxed out iterations (" << m_maxIterations << ")";

	return ccdState;
}

bool CcdCollisionLoop::findEarliestContact(
	const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs,
	double* currentTimeOfImpact)
{
	SURGSIM_ASSERT(currentTimeOfImpact != nullptr) << "Please provide a valid currentTimeOfImpact variable";

	double timeOfImpact = std::numeric_limits<double>::max();

	// Find earliest time of impact
	for (auto& pair : ccdPairs)
	{
		std::for_each(pair->getContacts().begin(),
					  pair->getContacts().end(),
					  [&timeOfImpact](const std::shared_ptr<Collision::Contact>& contact)
		{
			timeOfImpact = std::min<double>(timeOfImpact, contact->time);
		});
	}

	// Did not find any contacts return false
	if (!(timeOfImpact < std::numeric_limits<double>::max()))
	{
		return false;
	}

	*currentTimeOfImpact = timeOfImpact;

	return true;
}

void CcdCollisionLoop::filterLaterContacts(
	std::vector<std::shared_ptr<Collision::CollisionPair>>* ccdPairs,
	double epsilon,
	double timeOfImpact)
{
	for (auto& pair : (*ccdPairs))
	{
		pair->getContacts().remove_if([timeOfImpact, epsilon](const std::shared_ptr<Collision::Contact>& contact)
		{
			return contact->time > timeOfImpact + epsilon;
		});
	}
}

void CcdCollisionLoop::backupContacts(std::vector<std::shared_ptr<Collision::CollisionPair>>* ccdPairs,
									  std::vector<std::list<std::shared_ptr<Collision::Contact>>>* oldContacts)
{
	SURGSIM_ASSERT(oldContacts != nullptr) << "Invalid container found.";
	for (auto& pair : (*ccdPairs))
	{
		oldContacts->push_back(std::move(pair->getContacts()));
		pair->getContacts().clear();
	}
}

void CcdCollisionLoop::restoreContacts(std::vector<std::shared_ptr<Collision::CollisionPair>>* ccdPairs,
									   std::vector<std::list<std::shared_ptr<Collision::Contact>>>* oldContacts)
{
	SURGSIM_ASSERT(oldContacts != nullptr) << "Invalid container found.";
	if (oldContacts->size() == 0)
	{
		return;
	}

	SURGSIM_ASSERT(oldContacts->size() == ccdPairs->size()) << "Contact size exception detected";
	for (size_t i = 0; i < oldContacts->size(); ++i)
	{
		auto& newContacts = ccdPairs->at(i)->getContacts();
		newContacts.splice(newContacts.end(), std::move(oldContacts->at(i)));
	}
	oldContacts->clear();
}

void CcdCollisionLoop::printContacts(const std::vector<std::shared_ptr<Collision::CollisionPair>>& ccdPairs)
{
	std::stringstream out;
	size_t contactCount = 0;
	for (const auto& pair : ccdPairs)
	{
		for (const auto& contact : pair->getContacts())
		{
			out << *contact;
			contactCount++;
		}
	}
	SURGSIM_LOG_IF(contactCount != 0, m_logger, DEBUG) << "Number of Contacts: " <<
			contactCount << std::endl << out.str();
}
}
}
