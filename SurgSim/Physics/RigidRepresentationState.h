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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATIONSTATE_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATIONSTATE_H

#include "SurgSim/Physics/RigidRepresentationBaseState.h"

#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

/// The RigidRepresentationState class describes a dynamic rigid body state
/// (position + velocity information)
/// Derives from RigidRepresentationBaseState which describes the position state only
class RigidRepresentationState : public RigidRepresentationBaseState
{
public:
	/// Default constructor
	RigidRepresentationState()
		: RigidRepresentationBaseState()
	{
		m_v.setZero();
		m_w.setZero();
	}

	/// Destructor
	virtual ~RigidRepresentationState()
	{
	}

	/// Comparison operator
	/// \param s A RigidRepresentationState to compare it to
	/// \return True if the 2 states are equals, False otherwise
	bool operator ==(const RigidRepresentationState &s) const
	{
		return (RigidRepresentationBaseState::operator ==(s) && m_v == s.m_v && m_w == s.m_w);
	}

	/// Comparison operator
	/// \param s A RigidRepresentationState to compare it to
	/// \return False if the 2 states are equals, True otherwise
	bool operator !=(const RigidRepresentationState &s) const
	{
		return ! ((*this) == s);
	}

	/// Reset the state to default values
	/// Vectors will be filled with 0
	/// Rotations will be set to identity (quaternion or matrix type)
	/// If you want to reset to initial values, you need to save them separately
	/// in another RigidRepresentationState and assign it to this instance.
	virtual void reset()
	{
		RigidRepresentationBaseState::reset();

		m_v.setZero();
		m_w.setZero();
	}

	/// Get the linear velocity
	/// \return the linear velocity
	const Vector3d& getLinearVelocity() const
	{
		return m_v;
	}

	/// Get the angular velocity
	/// \return the angular velocity
	const Vector3d& getAngularVelocity() const
	{
		return m_w;
	}

	/// Set the linear velocity
	/// \param v The linear velocity
	void setLinearVelocity(const Vector3d &v)
	{
		m_v = v;
	}

	/// Set the angular velocity
	/// \param w The angular velocity
	void setAngularVelocity(const Vector3d &w)
	{
		m_w = w;
	}

private:
	/// Linear velocity
	Vector3d m_v;

	/// Angular velocity
	Vector3d m_w;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONSTATE_H
