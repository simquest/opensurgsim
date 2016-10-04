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

namespace SurgSim
{
namespace Framework
{
class Logger;
}

namespace Physics
{
class PhysicsManagerState;

class Representation;

/// Propagates the Mlcp result to the representations.
/// Can optionally discard (not push) MLCP results that do not meet the contact tolerance.
/// The MlcpGaussSeidelSolver has a contact tolerance that can be set through the SolveMlcp Computation, and which
/// is used to determine if the solver should stop before reaching the maximum iterations.  Often the SolveMlcp has a
/// "tight" (small) tolerance to get the most accurate contacts possible, but then the MLCP tends to iterate the
/// maximum number of times.  Then, here a looser (larger) tolerance can be set such that if the MLCP is failing to
/// meet that tolerance (i.e., the solve failed), the results are discarded.  Discarding the MLCP results will mean
/// the constraints will not be satisfied and may drive the simulation further away from successful MLCP results.
class PushResults : public Computation
{
public:
	/// Constructor
	/// \param doCopyState Specify if the output state in Computation::Update() is a copy or not of the input state
	explicit PushResults(bool doCopyState = false);

	SURGSIM_CLASSNAME(SurgSim::Physics::PushResults);

	/// Destructor
	virtual ~PushResults();

	/// Enable/disable discarding all MLCP results if any contact constraint has a violation greater than the tolerance.
	/// \param filter true to enable filtering.
	void setDiscardBadResults(bool filter);

	/// \return true if discarding MLCP results that have a contact violation greater than the tolerance.
	bool isDiscardBadResults() const;

	/// \param tolerance The contact tolerance limit, only matters if discarding bad MLCP results.
	void setContactTolerance(double tolerance);

	/// \return The contact tolerance.
	/// \sa MlcpGaussSeidelSolver::getContactTolerance
	double getContactTolerance() const;

protected:
	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
	override;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

private:
	/// The contact tolerance.
	/// \sa MlcpGaussSeidelSolver::m_contactTolerance
	double m_contactTolerance = 1e300;

	/// If true, does not push MLCP results if any contact is outside of the tolerance.
	bool m_discardBadResults = false;
};

}; // namespace Physics
}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_PUSHRESULTS_H
