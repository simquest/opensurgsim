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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASESTATE_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASESTATE_H

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{

namespace Physics
{

/// The RigidRepresentationBaseState class describes the common rigid body state
/// shared among all rigid objects (dynamic, static or fixed)
class RigidRepresentationBaseState : public SurgSim::Framework::Accessible
{
public:
	/// Default constructor
	RigidRepresentationBaseState();

	/// Copy constructor
	/// \param rhs Right hand side RigidRepresentationBaseState used to initialize a new RigidRepresentationBaseState.
	RigidRepresentationBaseState(const RigidRepresentationBaseState& rhs);

	/// Copy assignment
	/// \param rhs Right hand side RigidRepresentationBaseState from which data are copied.
	/// \note 'm_functors' in base class Accessible is NOT copied.
	RigidRepresentationBaseState& operator=(const RigidRepresentationBaseState& rhs);

	/// Destructor
	virtual ~RigidRepresentationBaseState();

	SURGSIM_CLASSNAME(SurgSim::Physics::RigidRepresentationBaseState);

	/// Comparison operator
	/// \param rhs A RigidRepresentationBaseState to compare it to
	/// \return True if the 2 states are equals, False otherwise
	bool operator==(const RigidRepresentationBaseState& rhs) const;

	/// Comparison operator
	/// \param rhs A RigidRepresentationBaseState to compare it to
	/// \return False if the 2 states are equals, True otherwise
	bool operator!=(const RigidRepresentationBaseState& rhs) const;

	/// Reset the state to default values
	/// Pose is being set to identity (no translation, no rotation)
	virtual void reset();

	/// Set the rigid representation pose
	/// \param pose The pose to set the rigid representation to
	void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Get the rigid representation pose
	/// \return A constant reference to the pose (read only)
	const SurgSim::Math::RigidTransform3d& getPose() const;

private:
	/// Register accessors of serializable properties
	void addSerializableProperty();

	/// Rigid representation pose (translation + rotation)
	SurgSim::Math::RigidTransform3d m_pose;
};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASESTATE_H