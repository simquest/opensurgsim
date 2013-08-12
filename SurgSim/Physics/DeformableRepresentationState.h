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
/// It contains positions and velocities for all degrees of freedom
class DeformableRepresentationState
{
public:
	/// Default constructor
	DeformableRepresentationState()
	{
	}

	/// Destructor
	~DeformableRepresentationState()
	{
	}

	/// Comparison operator (equality test)
	/// \param state The state to compare it to
	/// \return True if the 2 states are equal, False otherwise
	bool operator ==(const DeformableRepresentationState& state) const
	{
		return m_x == state.m_x && m_v == state.m_v;
	}

	/// Comparison operator (difference test)
	/// \param state The state to compare it to
	/// \return False if the 2 states are equal, True otherwise
	bool operator !=(const DeformableRepresentationState& state) const
	{
		return ! ((*this) == state);
	}

	/// Reset the state
	/// \note Simply set all positions/velocities to 0
	void reset()
	{
		m_x.setZero();
		m_v.setZero();
	}

	/// Allocate the state for a given number of degrees of freedom
	/// \param numDof The number of degrees of freedom to account for
	void allocate(int numDof)
	{
		m_x.resize(numDof);
		m_v.resize(numDof);
	}

	/// Retrieve position of all degrees of freedom
	/// \return A vector of all positions
	Eigen::Matrix<double, Eigen::Dynamic, 1>& getPositions()
	{
		return m_x;
	}

	/// Retrieve velocity of all degrees of freedom
	/// \return A vector of all velocities
	Eigen::Matrix<double, Eigen::Dynamic, 1>& getVelocities()
	{
		return m_v;
	}

private:

	/// State on the position level
	Eigen::Matrix<double, Eigen::Dynamic, 1> m_x;

	/// State on the velocity level
	Eigen::Matrix<double, Eigen::Dynamic, 1> m_v;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONSTATE_H
