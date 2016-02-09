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

#ifndef SURGSIM_PHYSICS_COMPUTATIONGROUP_H
#define SURGSIM_PHYSICS_COMPUTATIONGROUP_H

#include "SurgSim/Physics/Computation.h"
#include "SurgSim/Framework/Macros.h"

#include "boost/thread.hpp"

namespace SurgSim
{

namespace Physics
{

class ComputationGroup : public Computation
{
public:
	/// Constructor
	ComputationGroup(bool copyState);

	/// Destructor
	~ComputationGroup();

	SURGSIM_CLASSNAME(SurgSim::Physics::ComputationGroup);

	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt,
			const std::shared_ptr<PhysicsManagerState>& state) override;

	void addComputation(const std::shared_ptr<Computation>& computation);

	virtual bool endLoop();
private:

	std::vector<std::shared_ptr<Computation>> m_computations;
	std::function<bool(std::shared_ptr<PhysicsManagerState>)> m_breakLoop;

	boost::mutex m_computationsMutex;
};

}
}

#endif
