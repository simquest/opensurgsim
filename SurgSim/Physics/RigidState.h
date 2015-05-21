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

#ifndef SURGSIM_PHYSICS_RIGIDSTATE_H
#define SURGSIM_PHYSICS_RIGIDSTATE_H

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

/// The RigidState class describes a state (position and velocity information).
class RigidState : public SurgSim::Framework::Accessible
{
public:
	/// Default constructor
	RigidState();

	/// Default constructor
	RigidState(const RigidState& rhs);

	/// Copy assignment
	/// \param rhs Right hand side RigidState from which data are copied.
	/// \note 'm_functors' in base class Accessible is NOT copied.
	RigidState& operator=(const RigidState& rhs);

	/// Destructor
	virtual ~RigidState();

	SURGSIM_CLASSNAME(SurgSim::Physics::RigidState);

	/// Comparison operator
	/// \param rhs A RigidState to compare it to
	/// \return True if the 2 states are equals, False otherwise
	bool operator==(const RigidState& rhs) const;

	/// Comparison operator
	/// \param rhs A RigidState to compare it to
	/// \return False if the 2 states are equals, True otherwise
	bool operator!=(const RigidState& rhs) const;

	/// Reset the state to default values
	/// Vectors will be filled with 0
	/// Rotations will be set to identity (quaternion or matrix type)
	/// If you want to reset to initial values, you need to save them separately
	/// in another RigidState and assign it to this instance.
	void reset();

	/// Get the linear velocity
	/// \return the linear velocity
	const SurgSim::Math::Vector3d& getLinearVelocity() const;

	/// Get the angular velocity
	/// \return the angular velocity
	const SurgSim::Math::Vector3d& getAngularVelocity() const;

	/// Set the linear velocity
	/// \param v The linear velocity
	void setLinearVelocity(const SurgSim::Math::Vector3d &v);

	/// Set the angular velocity
	/// \param w The angular velocity
	void setAngularVelocity(const SurgSim::Math::Vector3d &w);

	/// Set the rigid representation pose
	/// \param pose The pose to set the rigid representation to
	void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Get the rigid representation pose
	/// \return A constant reference to the pose (read only)
	const SurgSim::Math::RigidTransform3d& getPose() const;

private:
	/// Register accessors of serializable properties
	void addSerializableProperty();

	/// Linear velocity
	SurgSim::Math::Vector3d m_v;

	/// Angular velocity
	SurgSim::Math::Vector3d m_w;

	/// Rigid representation pose (translation + rotation)
	SurgSim::Math::RigidTransform3d m_pose;
};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDSTATE_H