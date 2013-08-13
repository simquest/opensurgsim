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

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONSTATE_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONSTATE_H

#include <SurgSim/Math/Vector.h>

namespace SurgSim
{

namespace Physics
{

/// Defines the state for all deformable representations
/// It contains a collecion of position and velocity for all degrees of freedom
class DeformableRepresentationState
{
public:
	/// Default constructor
	DeformableRepresentationState();

	/// Destructor
	~DeformableRepresentationState();

	/// Comparison operator (equality test)
	/// \param state The state to compare it to
	/// \return True if the 2 states are equal, False otherwise
	bool operator ==(const DeformableRepresentationState& state) const;

	/// Comparison operator (difference test)
	/// \param state The state to compare it to
	/// \return False if the 2 states are equal, True otherwise
	bool operator !=(const DeformableRepresentationState& state) const;

	/// Reset the state
	/// \note Simply set all positions/velocities to 0
	void reset();

	/// Allocate the state for a given number of degrees of freedom
	/// \param numDof The number of degrees of freedom to account for
	void allocate(int numDof);

	/// Retrieve all degrees of freedom's position
	/// \return Vector of collected DOF's position
	Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& getPositions();

	/// Retrieve all degrees of freedom's position
	/// \return Vector of collected DOF's position
	const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& getPositions() const;

	/// Retrieve all degrees of freedom's velocity
	/// \return Vector of collected DOF's velocity
	Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& getVelocities();

	/// Retrieve all degrees of freedom's velocity
	/// \return Vector of collected DOF's velocity
	const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& getVelocities() const;

private:
	/// Default public copy constructor and assignment operator are being used on purpose

	/// State on the position level
	Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign> m_x;

	/// State on the velocity level
	Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign> m_v;
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONSTATE_H
