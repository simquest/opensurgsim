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

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

namespace SurgSim
{

namespace Physics
{

class DeformableRepresentationState;

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

	/// Gets the number of nodes the spring is connecting
	/// \return The number of nodes
	unsigned int getNumNodes() const
	{
		return m_nodeIds.size();
	}

	/// Gets the springNodeId-th node id
	/// \return The requested node id
	unsigned int getNodeId(unsigned int springNodeId) const
	{
		return m_nodeIds[springNodeId];
	}

	/// Gets the node ids for this spring
	/// \return A vector containing the node ids on which the spring is attached
	const std::vector<unsigned int>& getNodeIds() const
	{
		return m_nodeIds;
	}

	/// Computes the spring force from a given state
	/// \param state The state to compute the force with
	/// \return The computed spring force of size (3 x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	virtual const SurgSim::Math::Vector& computeForce(const DeformableRepresentationState& state) = 0;

	/// Computes the spring damping matrix D (= -df/dv) from a given state
	/// \param state The state to compute the damping matrix with
	/// \return The computed spring damping matrix  of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	virtual const SurgSim::Math::Matrix& computeDamping(const DeformableRepresentationState& state) = 0;

	/// Computes the spring stiffness matrix K (= -df/dx) from a given state
	/// \param state The state to compute the stiffness matrix with
	/// \return The computed spring stiffness matrix of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	virtual const SurgSim::Math::Matrix& computeStiffness(const DeformableRepresentationState& state) = 0;

	/// Computes the spring force vector as well as stiffness and damping matrices from a given state
	/// \param state The state to compute everything with
	/// \param[out] f The computed spring force of size (3 x getNumNodes())
	/// \param[out] D The computed spring damping matrix of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \param[out] K The computed spring stiffness matrix of size (3 x getNumNodes(), 3 x getNumNodes())
	/// \note (non-const) because it uses internal data members to store and return the results
	virtual void computeFDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector** f, SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K) = 0;

protected:
	/// Node ids connected by this spring
	std::vector<unsigned int> m_nodeIds;

	/// Force vector (held internally to avoid re-allocation)
	SurgSim::Math::Vector m_f;

	/// Stiffness matrix (held internally to avoid re-allocation)
	SurgSim::Math::Matrix m_K;

	/// Damping matrix (held internally to avoid re-allocation)
	SurgSim::Math::Matrix m_D;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_SPRING_H
