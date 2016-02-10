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

#include <boost/thread.hpp>

namespace SurgSim
{

namespace Physics
{

/// Implements a mechanism to group and loop computations, the computations will be called in sequence repeatedly
/// until one of the exit criteria is met. That is either the endIteration() call, or shouldAbortLoop()
/// in the \sa PhysicsManagerState return true.
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

	/// Override this function to implement a custom criterion to exit this computation, when this returns true
	/// the computation will exit and return the last state
	/// \note the computation can also be stopped by calling setAbortLoop(true) on the physics state by any calculation
	/// \return true to stop calculating.
	virtual bool endIteration();

	/// Adds a computation to this group, the computation will be appended at the end
	/// \param computation the computation to be added to this group
	void addComputation(const std::shared_ptr<Computation>& computation);

	/// \return the computations used by this class
	std::vector<std::shared_ptr<Computation>> getComputations() const;

	/// \param set the computations used by this class
	void setComputations(const std::vector<std::shared_ptr<Computation>>& val);

private:

	size_t m_iterations;
	std::vector<std::shared_ptr<Computation>> m_computations;
	mutable boost::mutex m_computationsMutex;


};

}
}

#endif
