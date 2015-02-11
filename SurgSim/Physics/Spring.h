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

#ifndef SURGSIM_PHYSICS_SPRING_H
#define SURGSIM_PHYSICS_SPRING_H

#include <vector>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{
class OdeState;
};

namespace Physics
{

/// Base class for all springs
/// It handles the node ids to which it is connected and requires all derived classes
/// to compute the force and its derivatives (the stiffness and damping matrices)
/// A extra method also exist to compute all of them at once for performance purposes.
/// It holds on to the actual computed values (m_f, m_K, m_D) as its size is not predefined from outside
/// and would requires intensive (de)allocation or a temporary variable anyway
class Spring
{
public:
	/// Virtual destructor
	virtual ~Spring()
	{}

	/// Initialize the Spring once everything has been set
	/// \param state The state to initialize the Spring with
	virtual void initialize(const SurgSim::Math::OdeState& state)
	{
	}

	/// Gets the number of nodes the spring is connecting
	/// \return The number of nodes
	size_t getNumNodes() const
	{
		return m_nodeIds.size();
	}

	/// Gets the springNodeId-th node id
	/// \return The requested node id
	size_t getNodeId(size_t springNodeId) const
	{
		return m_nodeIds[springNodeId];
	}

	/// Gets the node ids for this spring
	/// \return A vector containing the node ids on which the spring is attached
	const std::vector<size_t>& getNodeIds() const
	{
		return m_nodeIds;
	}

	/// Adds the spring force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the spring force into
	/// \param scale A factor to scale the added force with
	virtual void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
						  double scale = 1.0) = 0;

	/// Adds the spring damping matrix D (= -df/dv) (computed for a given state) to a complete system damping matrix
	/// D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the spring damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	virtual void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* D,
							double scale = 1.0) = 0;

	/// Adds the spring stiffness matrix K (= -df/dx) (computed for a given state) to a complete system stiffness
	/// matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the spring stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	virtual void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K,
							  double scale = 1.0) = 0;

	/// Adds the spring force vector, mass, stiffness and damping matrices (computed for a given state) into a
	/// complete system data structure F, D, K (assembly)
	/// \param state The state to compute everything with
	/// \param[in,out] F The complete system force vector to add the spring force into
	/// \param[in,out] D The complete system damping matrix to add the spring damping matrix into
	/// \param[in,out] K The complete system stiffness matrix to add the spring stiffness matrix into
	virtual void addFDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
						 SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K) = 0;

	/// Adds the spring matrix-vector contribution F += (alphaD.D + alphaK.K).x (computed for a given
	/// state) into a complete system data structure F (assembly)
	/// \param state The state to compute everything with
	/// \param alphaD The scaling factor for the damping contribution
	/// \param alphaK The scaling factor for the stiffness contribution
	/// \param x A complete system vector to use as the vector in the matrix-vector multiplication
	/// \param[in,out] F The complete system force vector to add the element matrix-vector contribution into
	virtual void addMatVec(const SurgSim::Math::OdeState& state, double alphaD, double alphaK,
						   const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) = 0;

protected:
	/// Node ids connected by this spring
	std::vector<size_t> m_nodeIds;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_SPRING_H
