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

#ifndef SURGSIM_PHYSICS_PUSHRESULTS_H
#define SURGSIM_PHYSICS_PUSHRESULTS_H

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Physics/Computation.h"

namespace Framework
{
class Logger;
}

namespace SurgSim
{
namespace Physics
{

class Representation;

/// Propagates the Mlcp result to the representations
class PushResults : public Computation
{
public:
	/// Constructor
	/// \param doCopyState Specify if the output state in Computation::Update() is a copy or not of the input state
	explicit PushResults(bool doCopyState = false);

	SURGSIM_CLASSNAME(SurgSim::Physics::PushResults);

	/// Destructor
	virtual ~PushResults();

protected:

	/// Override doUpdate from superclass
	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
	override;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

}; // namespace Physics
}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_PUSHRESULTS_H
