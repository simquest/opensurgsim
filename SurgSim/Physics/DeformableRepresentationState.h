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

#include <vector>

#include <SurgSim/Math/Vector.h>

namespace SurgSim
{

namespace Physics
{

/// Defines the state for all deformable representations.
/// It contains collections of position, velocity and acceleration for all degrees of freedom
/// It also contains the boundary conditions (per degree of freedom) associated to this state
/// A Degree of freedom (DOF) is a SINGLE INDEPENDENT parameter for defining the configuration of the model.
/// Example 1: A mass in 3D space has 3DOF (independent coordinates along axis X, Y and Z).
/// Example 2: A beam element model is a 6DOF model (independent coordinates along axis X, Y and Z,
///                                           and independent rotation parameters along X, Y and Z).
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

	/// Resets the state
	/// \note Simply set all positions/velocities/accelerations to 0 and remove all boundary conditions
	void reset();

	/// Allocates the state for a given number of degrees of freedom
	/// \param numDofPerNode The number of degrees of freedom per node to account for
	/// \param numNodes The number of nodes to account for
	/// \note This method clears all the data structures and remove all existing boundary conditions
	void setNumDof(unsigned int numDofPerNode, unsigned int numNodes);

	/// Retrieves the number of degrees of freedom
	/// \return The number of DOF for this deformable representation
	unsigned int getNumDof() const;

	/// Retrieves the number of nodes
	/// \return The number of nodes for this deformable representation
	unsigned int getNumNodes() const;

	/// Retrieves all degrees of freedom's position (non-const version)
	/// \return Vector of collected DOF's position
	SurgSim::Math::Vector& getPositions();

	/// Retrieves all degrees of freedom's position (const version)
	/// \return Vector of collected DOF's position
	const SurgSim::Math::Vector& getPositions() const;

	/// Retrieves the position of a given node (const version)
	/// \param nodeId The desired node id for which the position is requested (must be a valid id)
	/// \return The position of the node nodeId
	/// \note Behavior undefined if the nodeId is not in the correct range [0 getNumNodes()-1]
	const SurgSim::Math::Vector3d getPosition(unsigned int nodeId) const;

	/// Retrieves all degrees of freedom's velocity (non-const version)
	/// \return Vector of collected DOF's velocity
	SurgSim::Math::Vector& getVelocities();

	/// Retrieves all degrees of freedom's velocity (const version)
	/// \return Vector of collected DOF's velocity
	const SurgSim::Math::Vector& getVelocities() const;

	/// Retrieves the velocity of a given node (const version)
	/// \param nodeId The desired node id for which the velocity is requested (must be a valid id)
	/// \return The velocity of the node nodeId
	/// \note Behavior undefined if the nodeId is not in the correct range [0 getNumNodes()-1]
	const SurgSim::Math::Vector3d getVelocity(unsigned int nodeId) const;

	/// Retrieves all degrees of freedom's acceleration (non-const version)
	/// \return Vector of collected DOF's acceleration
	SurgSim::Math::Vector& getAccelerations();

	/// Retrieves all degrees of freedom's acceleration (const version)
	/// \return Vector of collected DOF's acceleration
	const SurgSim::Math::Vector& getAccelerations() const;

	/// Retrieves the acceleration of a given node (const version)
	/// \param nodeId The desired node id for which the acceleration is requested (must be a valid id)
	/// \return The acceleration of the node nodeId
	/// \note Behavior undefined if the nodeId is not in the correct range [0 getNumNodes()-1]
	const SurgSim::Math::Vector3d getAcceleration(unsigned int nodeId) const;

	/// Adds a boundary condition on a given dof
	/// \param dof The dof to set as a boundary condition
	/// \note No test is performed on dof, the behavior is undefined when dof is out of range
	void addBoundaryCondition(unsigned int dof);

	/// Retrieves the number of boundary conditions
	/// \return The number of boundary conditions
	unsigned int getNumBoundaryConditions() const;

	/// Retrieves all boundary conditions
	/// \return All boundary conditions as a vector of dof ids
	const std::vector<unsigned int>& getBoundaryConditions() const;

	/// Queries if a specific dof is a boundary condition or not
	/// \param dof The requested dof
	/// \return True if dof is a boundary condition, False otherwise
	/// \note The behavior is undefined when dof is out of range [0 getNumBoundaryConditions()-1]
	bool isBoundaryCondition(unsigned int dof) const;

private:
	/// Default public copy constructor and assignment operator are being used on purpose

	/// Keep track of the number of degrees of freedom per node and the number of nodes
	unsigned int m_numDofPerNode, m_numNodes;

	/// Degrees of freedom position
	SurgSim::Math::Vector m_x;

	/// Degrees of freedom velocity (m_x 1st derivative w.r.t. time)
	SurgSim::Math::Vector m_v;

	/// Degrees of freedom acceleration (m_x 2nd derivative w.r.t. time)
	SurgSim::Math::Vector m_a;

	/// Boundary conditions stored as a list of dof ids
	std::vector<unsigned int> m_boundaryConditionsAsDofIds;

	/// Boundary conditions stored per dof (True indicates a boundary condition, False does not)
	Eigen::Matrix<bool, Eigen::Dynamic, 1> m_boundaryConditionsPerDof;
};

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONSTATE_H
