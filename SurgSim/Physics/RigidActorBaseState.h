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

#ifndef SURGSIM_PHYSICS_RIGIDACTORBASESTATE_H
#define SURGSIM_PHYSICS_RIGIDACTORBASESTATE_H

#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

/// The RigidActorBaseState class describes the common rigid body state
/// shared among all rigid objects (dynamic, static or fixed)
class RigidActorBaseState
{
public:
	/// Default constructor
	RigidActorBaseState()
	{
		m_pose.setIdentity();
	}

	/// Destructor
	virtual ~RigidActorBaseState()
	{
	}

	/// Comparison operator
	/// \param state A RigidActorBaseState to compare it to
	/// \return True if the 2 states are equals, False otherwise
	bool operator ==(const RigidActorBaseState& state) const
	{
		return m_pose.isApprox(state.m_pose);
	}

	/// Reset the state to default values
	/// Pose is being set to identity (no translation, no rotation)
	virtual void reset()
	{
		m_pose.setIdentity();
	}

	/// Set the rigid actor pose
	/// \param pose The pose to set the rigid actor to
	void setPose(const SurgSim::Math::RigidTransform3d& pose)
	{
		m_pose = pose;
	}

	/// Get the rigid actor pose
	/// \return A constant reference to the pose (read only)
	const SurgSim::Math::RigidTransform3d& getPose() const
	{
		return m_pose;
	}

private:
	/// Rigid actor pose (translation + rotation)
	SurgSim::Math::RigidTransform3d m_pose;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDACTORBASESTATE_H
