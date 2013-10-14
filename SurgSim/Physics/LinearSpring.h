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

#ifndef SURGSIM_PHYSICS_LINEARSPRING_H
#define SURGSIM_PHYSICS_LINEARSPRING_H

#include <SurgSim/Physics/Spring.h>

namespace SurgSim
{

namespace Physics
{

/// Linear spring connecting 2 nodes with a viscous term
class LinearSpring : public Spring
{
public:
	/// Constructor
	/// \param nodeId0, nodeId1 The node ids on which the spring is attached
	LinearSpring(unsigned int nodeId0, unsigned int nodeId1);

	/// Sets the spring stiffness parameter
	/// \param stiffness The stiffness to assign to the spring (in N.m-1)
	void setStiffness(double stiffness);

	/// Gets the spring stiffness parameter
	/// \return The stiffness assigned to the spring (in N.m-1)
	double getStiffness() const;

	/// Sets the spring damping parameter
	/// \param damping The damping to assign to the spring (in N.s.m-1)
	void setDamping(double damping);

	/// Gets the spring damping parameter
	/// \return The damping assigned to the spring (in N.s.m-1)
	double getDamping() const;

	/// Sets the rest length of the spring
	/// \param l0 The rest length to assign to the spring (in m)
	void setInitialLength(double l0);

	/// Gets the rest length of the spring
	/// \return The rest length assigned to the spring (in m)
	double getInitialLength() const;

	/// Computes the spring force from a given state
	/// \param state The state to compute the force with
	/// \return The computed spring force of size (3 x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	const SurgSim::Math::Vector& computeForce(const DeformableRepresentationState& state) override;

	/// Computes the spring stiffness matrix K (= -df/dx) from a given state
	/// \param state The state to compute the stiffness matrix with
	/// \return The computed spring stiffness matrix of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	const SurgSim::Math::Matrix& computeStiffness(const DeformableRepresentationState& state) override;

	/// Computes the spring damping matrix D (= -df/dv) from a given state
	/// \param state The state to compute the damping matrix with
	/// \return The computed spring damping matrix  of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	const SurgSim::Math::Matrix& computeDamping(const DeformableRepresentationState& state) override;

	/// Computes the spring force vector as well as stiffness and damping matrices from a given state
	/// \param state The state to compute everything with
	/// \param[out] f The computed spring force of size (3 x getNumNodes())
	/// \param[out] D The computed spring damping matrix of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \param[out] K The computed spring stiffness matrix of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \note (non-const) because it uses internal data members to store and return the results
	void computeFDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector** f, SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K);

	/// Comparison operator (equality)
	/// \param spring Spring to compare it to
	/// \return True if the 2 springs contains the same information, false otherwise
	/// \note Comparison is based on spring type, rest length, stiffness and damping coefficients ONLY
	bool operator ==(const Spring& spring) const;

	/// Comparison operator (inequality)
	/// \param spring Spring to compare it to
	/// \return False if the 2 springs contains the same information, true otherwise
	/// \note Comparison is based on spring type, rest length, stiffness and damping coefficients ONLY
	bool operator !=(const Spring& spring) const;

private:
	/// Rest length (in m)
	double m_l0;

	/// Stiffness parameters (in N.m-1)
	double m_stiffness;

	/// Damping parameters (in N.s.m-1)
	double m_damping;
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_LINEARSPRING_H
