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

#ifndef SURGSIM_PHYSICS_CONTACTFILTERING_H
#define SURGSIM_PHYSICS_CONTACTFILTERING_H

#include <memory>

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/Computation.h"

namespace SurgSim
{

namespace Physics
{
class PhysicsManagerState;

/// Computation to filter the contacts with the relative motion. Any contact orthogonal to the motion will be filtered.
class ContactFiltering : public Computation
{
public:
	/// Constructor
	/// \param doCopyState Specify if the output state in Computation::Update() is a copy or not of the input state
	explicit ContactFiltering(bool doCopyState = false);

	SURGSIM_CLASSNAME(SurgSim::Physics::ContactFiltering);

	/// Destructor
	virtual ~ContactFiltering();

	/// \param angleLimit The limit angle from which the contacts are filtered
	/// \note Noting 'n' the contact normal and 'v' the relative velocity
	/// \note A contact is filtered if |n.v| < cos(angleLimit)
	void setAngleLimit(double angleLimit);

	/// \return The limit angle (between the contact normal and the relative velocity) from which the contacts are filtered
	/// \note Noting 'n' the contact normal and 'v' the relative velocity
	/// \note A contact is filtered if |n.v| < cos(angleLimit)
	/// \note Default is PI/2, which means no contacts are filtered
	double getAngleLimit() const;

protected:
	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
	override;

private:
	/// The logger for this class
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// Angle limit (between contact normal and relative velocity) at which contacts get filtered
	double m_angleLimit;

};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_CONTACTFILTERING_H
