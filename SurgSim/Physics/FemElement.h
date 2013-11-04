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

#ifndef SURGSIM_PHYSICS_FEMELEMENT_H
#define SURGSIM_PHYSICS_FEMELEMENT_H

#include <vector>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

namespace SurgSim
{

namespace Physics
{

class DeformableRepresentationState;

/// Base class for all Fem Element (1D, 2D, 3D)
/// It handles the node ids to which it is connected and requires all derived classes to compute the element
/// mass matrix and the force vector along with the derivatives (the stiffness and damping matrices).
/// A extra method also exist to compute all of them at once for performance purposes.
/// It holds on to the actual computed values (m_f, m_M, m_D, m_K) as its size is not predefined from outside
/// and would requires intensive (de)allocation or a temporary variable anyway.
class FemElement
{
public:
	/// Virtual destructor
	virtual ~FemElement()
	{}

	/// Gets the number of degree of freedom per node
	/// \return The number of dof per node
	unsigned int getNumDofPerNode() const
	{
		return m_numDofPerNode;
	}

	/// Gets the number of nodes connected by this element
	/// \return The number of nodes
	unsigned int getNumNodes() const
	{
		return m_nodeIds.size();
	}

	/// Gets the elementNodeId-th node id
	/// \return The requested node id
	unsigned int getNodeId(unsigned int elementNodeId) const
	{
		return m_nodeIds[elementNodeId];
	}

	/// Gets the node ids for this element
	/// \return A vector containing the node ids on which the element is defined
	const std::vector<unsigned int>& getNodeIds() const
	{
		return m_nodeIds;
	}

	/// Get the element volume based on the input state
	/// \param state The deformable state to compute the volume with
	virtual double getVolume(const DeformableRepresentationState& state) const = 0;

	/// Computes the element force from a given state
	/// \param state The state to compute the force with
	/// \return The computed element force of size (getNumDofPerNode() x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number
	/// \note of dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Vector& computeForce(const DeformableRepresentationState& state) = 0;

	/// Computes the element mass matrix M from a given state
	/// \param state The state to compute the mass matrix with
	/// \return The computed element square mass matrix of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number
	/// \note of dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Matrix& computeMass(const DeformableRepresentationState& state) = 0;

	/// Computes the element damping matrix D (= -df/dv) from a given state
	/// \param state The state to compute the damping matrix with
	/// \return The computed element square damping matrix  of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Matrix& computeDamping(const DeformableRepresentationState& state) = 0;

	/// Computes the element stiffness matrix K (= -df/dx) from a given state
	/// \param state The state to compute the stiffness matrix with
	/// \return The computed element square stiffness matrix of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Matrix& computeStiffness(const DeformableRepresentationState& state) = 0;

	/// Computes the element force vector, mass, stiffness and damping matrices from a given state
	/// \param state The state to compute everything with
	/// \param[out] f The computed element force of size (getNumDofPerNode() x getNumNodes())
	/// \param[out] M The computed element square mass of size getNumDofPerNode() x getNumNodes()
	/// \param[out] D The computed element square damping matrix of size getNumDofPerNode() x getNumNodes()
	/// \param[out] K The computed element square stiffness matrix of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses internal data members to store and return the results
	/// \note This method suppose that the incoming state contains information with the same number
	/// \note of dof per node as getNumDofPerNode()
	virtual void computeFMDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector** f, SurgSim::Math::Matrix** M, SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K) = 0;

protected:
	/// Allocate the internal data structures (vector m_f, matrices m_M, m_D, m_K)
	/// \return True if the allocation succeeded, False otherwise
	/// \note It relies on the methods getNumDofPerNode() and getNumNodes() to determine the size
	bool allocate()
	{
		using SurgSim::Math::resize;

		const unsigned int numDof = getNumDofPerNode() * getNumNodes();

		resize(&m_f, numDof, true);
		resize(&m_M, numDof, numDof, true);
		resize(&m_D, numDof, numDof, true);
		resize(&m_K, numDof, numDof, true);
		return static_cast<unsigned int>(m_f.size()) == numDof &&
			static_cast<unsigned int>(m_M.rows()) == numDof && static_cast<unsigned int>(m_M.cols()) == numDof &&
			static_cast<unsigned int>(m_D.rows()) == numDof && static_cast<unsigned int>(m_D.cols()) == numDof &&
			static_cast<unsigned int>(m_K.rows()) == numDof && static_cast<unsigned int>(m_K.cols()) == numDof;
	}

	/// Number of degree of freedom per node for this element
	unsigned int m_numDofPerNode;

	/// Node ids connected by this element
	std::vector<unsigned int> m_nodeIds;

	/// Force vector (held internally to avoid re-allocation)
	SurgSim::Math::Vector m_f;

	/// Mass matrix (held internally to avoid re-allocation)
	SurgSim::Math::Matrix m_M;

	/// Damping matrix (held internally to avoid re-allocation)
	SurgSim::Math::Matrix m_D;

	/// Stiffness matrix (held internally to avoid re-allocation)
	SurgSim::Math::Matrix m_K;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENT_H
