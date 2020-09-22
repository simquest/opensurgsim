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

#include "SurgSim/Physics/Spring.h"

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
	LinearSpring(size_t nodeId0, size_t nodeId1);

	void initialize(const SurgSim::Math::OdeState& state) override;

	/// Sets the spring stiffness parameter
	/// \param stiffness The stiffness to assign to the spring (in N.m-1)
	/// \exception SurgSim::Framework::AssertionFailure stiffness cannot be negative
	void setStiffness(double stiffness);

	/// Gets the spring stiffness parameter
	/// \return The stiffness assigned to the spring (in N.m-1)
	double getStiffness() const;

	/// Sets the spring damping parameter
	/// \param damping The damping to assign to the spring (in N.s.m-1)
	/// \exception SurgSim::Framework::AssertionFailure damping cannot be negative
	void setDamping(double damping);

	/// Gets the spring damping parameter
	/// \return The damping assigned to the spring (in N.s.m-1)
	double getDamping() const;

	/// Sets the rest length of the spring
	/// \param restLength The rest length to assign to the spring (in m)
	/// \exception SurgSim::Framework::AssertionFailure rest length cannot be negative
	void setRestLength(double restLength);

	/// Gets the rest length of the spring
	/// \return The rest length assigned to the spring (in m)
	double getRestLength() const;

	/// Adds the spring force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the spring force into
	/// \param scale A factor to scale the added force with
	void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale = 1.0) override;

	/// Adds the spring damping matrix D (= -df/dv) (computed for a given state) to a complete system damping matrix
	/// D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the spring damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* D, double scale = 1.0) override;

	/// Adds the spring stiffness matrix K (= -df/dx) (computed for a given state) to a complete system stiffness
	/// matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the spring stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* K,
					  double scale = 1.0) override;

	/// Adds the spring force vector, mass, stiffness and damping matrices (computed for a given state) into a
	/// complete system data structure F, D, K (assembly)
	/// \param state The state to compute everything with
	/// \param[in,out] F The complete system force vector to add the spring force into
	/// \param[in,out] D The complete system damping matrix to add the spring damping matrix into
	/// \param[in,out] K The complete system stiffness matrix to add the spring stiffness matrix into
	void addFDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
				SurgSim::Math::SparseMatrix* D, SurgSim::Math::SparseMatrix* K) override;

	/// Adds the spring matrix-vector contribution F += (alphaD.D + alphaK.K).x (computed for a given
	/// state) into a complete system data structure F (assembly)
	/// \param state The state to compute everything with
	/// \param alphaD The scaling factor for the damping contribution
	/// \param alphaK The scaling factor for the stiffness contribution
	/// \param vector A complete system vector to use as the vector in the matrix-vector multiplication
	/// \param[in,out] F The complete system force vector to add the element matrix-vector contribution into
	void addMatVec(const SurgSim::Math::OdeState& state, double alphaD, double alphaK,
				   const SurgSim::Math::Vector& vector, SurgSim::Math::Vector* F) override;

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

protected:
	/// Compute the stiffness matrix Ke = -dF1/dx1 and damping matrix De = -dF1/dv1 of this spring for a given state,
	/// where this spring is defined by its 2 nodes positions {x1, x2}, velocities {v1, v2} and forces {F1, F2=-F1}.
	/// \param state The state to compute the jacobians from
	/// \param [out] De, Ke Respectively the damping and stiffness matrices De and Ke
	/// \return True if the matrices could be computed, False otherwise (current length is null)
	/// \note This method calculates only the 3x3 parts related to the force applied on the first node,
	/// \note derived w.r.t. first node. By nature, we have dF2/dx2 = dF1/dx1 = -dF1/dx2 = -dF2/dx1.
	bool computeDampingAndStiffness(const SurgSim::Math::OdeState& state,
									SurgSim::Math::Matrix33d* De,
									SurgSim::Math::Matrix33d* Ke);

private:
	/// Rest length (in m)
	double m_restLength;

	/// Stiffness parameters (in N.m-1)
	double m_stiffness;

	/// Damping parameters (in N.s.m-1)
	double m_damping;
};

/// Helper method to create a LinearSpring
/// \param state The state to initialize the spring with (rest length calculation)
/// \param nodeId0, nodeId1 Node ids of the 2 connected masses
/// \param stiffness, damping The spring parameters
/// \return The newly create spring
std::shared_ptr<LinearSpring> createLinearSpring(const std::shared_ptr<Math::OdeState> state,
	size_t nodeId0, size_t nodeId1, double stiffness, double damping);

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_LINEARSPRING_H
