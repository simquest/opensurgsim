// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_MATH_ODESTATE_H
#define SURGSIM_MATH_ODESTATE_H

#include <memory>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{

/// The state \f$y\f$ of an ode of 2nd order of the form \f$M(x,v).a = F(x, v)\f$ with boundary conditions.
/// This ode equation is solved as an ode of order 1 by defining the state vector
/// \f$y = \left(\begin{array}{c}x\\v\end{array}\right)\f$:
/// \f[
///   y' = \left(\begin{array}{c} x' \\ v' \end{array}\right) =
///   \left(\begin{array}{c} v \\ M(x, v)^{-1}.F(x, v) \end{array}\right)
/// \f]
class OdeState
{
public:
	/// Default constructor
	OdeState();

	/// Destructor
	virtual ~OdeState();

	/// Comparison operator (equality test)
	/// \param state The state to compare it to
	/// \return True if the 2 states are equal, False otherwise
	bool operator ==(const OdeState& state) const;

	/// Comparison operator (difference test)
	/// \param state The state to compare it to
	/// \return False if the 2 states are equal, True otherwise
	bool operator !=(const OdeState& state) const;

	/// Resets the state
	/// \note Simply set all positions/velocities to 0 and remove all boundary conditions
	virtual void reset();

	/// Allocates the state for a given number of degrees of freedom
	/// \param numDofPerNode The number of degrees of freedom per node to account for
	/// \param numNodes The number of nodes to account for
	/// \note This method clears all the data structures and remove all existing boundary conditions
	virtual void setNumDof(size_t numDofPerNode, size_t numNodes);

	/// Retrieves the number of degrees of freedom
	/// \return The number of DOF for this representation
	size_t getNumDof() const;

	/// Retrieves the number of nodes
	/// \return The number of nodes for this representation
	size_t getNumNodes() const;

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
	const SurgSim::Math::Vector3d getPosition(size_t nodeId) const;

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
	const SurgSim::Math::Vector3d getVelocity(size_t nodeId) const;

	/// Adds boundary conditions for a given node (fixes all the dof for this node)
	/// \param nodeId The node to set the boundary conditions on
	void addBoundaryCondition(size_t nodeId);

	/// Adds a boundary condition on a given dof of a given node (only 1 dof is fixed)
	/// \param nodeId The node on which the boundary condition needs to be set
	/// \param nodeDofId The dof of the node to set as boundary condition
	void addBoundaryCondition(size_t nodeId, size_t nodeDofId);

	/// Retrieves the number of boundary conditions
	/// \return The number of boundary conditions
	size_t getNumBoundaryConditions() const;

	/// Retrieves all boundary conditions
	/// \return All boundary conditions as a vector of dof ids
	const std::vector<size_t>& getBoundaryConditions() const;

	/// Queries if a specific dof is a boundary condition or not
	/// \param dof The requested dof
	/// \return True if dof is a boundary condition, False otherwise
	/// \note The behavior is undefined when dof is out of range [0 getNumBoundaryConditions()-1]
	bool isBoundaryCondition(size_t dof) const;

	/// Apply boundary conditions to a given vector
	/// \param vector The vector to apply the boundary conditions on
	/// \return The parameter vector. This enables chained use like
	/// \f$U = K^1 * \text{applyBoundaryConditionsToVector}(x)\f$
	Vector* applyBoundaryConditionsToVector(Vector* vector) const;

	/// Apply boundary conditions to a given matrix
	/// \param matrix The dense matrix to apply the boundary conditions on
	/// \param hasCompliance True if the fixed dofs should have a compliance of 1 with themselves in the matrix or not.
	/// \note hasCompliance is practical to remove all compliance, which is helpful when the compliance matrix is used
	/// \note in an architecture of type LCP. It ensures that a separate constraint resolution will never violates the
	/// \note boundary conditions.
	void applyBoundaryConditionsToMatrix(Matrix* matrix, bool hasCompliance = true) const;

	/// Apply boundary conditions to a given matrix
	/// \param matrix The sparse matrix to apply the boundary conditions on
	/// \param hasCompliance True if the fixed dofs should have a compliance of 1 with themselves in the matrix or not.
	/// \note hasCompliance is practical to remove all compliance, which is helpful when the compliance matrix is used
	/// \note in an architecture of type LCP. It ensures that a separate constraint resolution will never violates the
	/// \note boundary conditions.
	void applyBoundaryConditionsToMatrix(SparseMatrix* matrix, bool hasCompliance = true) const;

	/// Adds a boundary condition on the static dof of a given node (we consider here that each node has a single static dof)
	/// \param nodeId The node on which the boundary condition needs to be set
	/// \param value The boundary conidtion value that the static dof needs to be set on
	/// \note for example, this is fixing the twist angle of a Kircchoff model using a quasi-static update of the material frame
	/// \note i.e. the twist is not part of the dynamic DOF, but is still a static dof of the model.
	void addBoundaryConditionStaticDof(size_t nodeId, double value);

	/// \return The number of boundary conditions for statically defined Dof
	size_t getNumBoundaryConditionsStaticDof() const;

	/// \return All boundary conditions for statically defined Dof as a vector of dof ids
	const std::vector<std::pair<size_t, double>>& getBoundaryConditionsStaticDof() const;

	/// Set all boundary conditions on static dofs
	/// \param staticDof The vector of static dof that must be set
	void setBoundaryConditionsStaticDof(const std::vector<std::pair<size_t, double>>& staticDof);

	/// Set the value of an existing boundary condition on a static dof
	/// \param nodeId The node for which the static dof must be set
	/// \param value The value to set the static dof of the given node
	/// \note Does not do anything if the static dof boundary condition does not exists yet.
	void setBoundaryConditionStaticDof(size_t nodeId, double value);

	/// Check if this state is numerically valid
	/// \return True if all positions and velocities are valid numerical values, False otherwise
	virtual bool isValid() const;

	/// Returns the linear interpolated ODE state between this and other at parameter t
	/// \param other the end point for the linear interpolation
	/// \param t the interpolation time
	/// \return the interpolated state = this + (other - this) * t;
	/// \note All dof are independently linearly interpolated (This will not work correctly
	/// on rotation vectors where a slerp will be required.)
	OdeState interpolate(const OdeState& other, double t) const;

private:
	/// Default public copy constructor and assignment operator are being used on purpose

	/// Keep track of the number of degrees of freedom per node and the number of nodes
	size_t m_numDofPerNode, m_numNodes;

	/// Degrees of freedom position
	SurgSim::Math::Vector m_x;

	/// Degrees of freedom velocity (m_x 1st derivative w.r.t. time)
	SurgSim::Math::Vector m_v;

	/// Boundary conditions stored as a list of dof ids
	std::vector<size_t> m_boundaryConditionsAsDofIds;

	/// Boundary conditions stored per dof (True indicates a boundary condition, False does not)
	Eigen::Matrix<bool, Eigen::Dynamic, 1> m_boundaryConditionsPerDof;

	/// Boundary conditions of non dof stored as a list of pairs of <node id, value>
	std::vector<std::pair<size_t, double>> m_boundaryConditionsStaticDof;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESTATE_H
