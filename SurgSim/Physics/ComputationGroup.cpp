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

#include "SurgSim/Physics/ComputationGroup.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

namespace SurgSim
{

namespace Physics
{

ComputationGroup::ComputationGroup(bool copyState) :
	Computation(copyState)
{

}

ComputationGroup::~ComputationGroup()
{

}

std::shared_ptr<PhysicsManagerState> ComputationGroup::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	boost::unique_lock<boost::mutex> lock(m_computationsMutex);
	m_iterations = 0;
	bool keepRunning = true;
	auto lastState = state;
	while (keepRunning)
	{
		++m_iterations;
		for (const auto& computation : m_computations)
		{
			lastState = computation->update(dt, lastState);
			if (lastState->shouldAbortGroup())
			{
				lastState->setAbortGroup(false);
				keepRunning = false;
				break;
			}
		}

		keepRunning = keepRunning && !endIteration();
	}
	return lastState;
}


void ComputationGroup::addComputation(const std::shared_ptr<Computation>& computation)
{
	boost::unique_lock<boost::mutex> lock(m_computationsMutex);
	m_computations.push_back(computation);
}

bool ComputationGroup::endIteration()
{
	return true;
}

std::vector<std::shared_ptr<Computation>> ComputationGroup::getComputations() const
{
	boost::unique_lock<boost::mutex> lock(m_computationsMutex);
	return m_computations;
}

void ComputationGroup::setComputations(const std::vector<std::shared_ptr<Computation>>& val)
{
	boost::unique_lock<boost::mutex> lock(m_computationsMutex);
	m_computations = val;
}

size_t ComputationGroup::getIterations()
{
	return m_iterations;
}

}
}