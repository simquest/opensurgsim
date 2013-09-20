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

#ifndef SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H
#define SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H

#include <set>

#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/MassSpringRepresentationState.h>
#include <SurgSim/DataStructures/TetrahedronMesh.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::DataStructures::TetrahedronMesh;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;

namespace SurgSim
{

namespace Physics
{

/// MassSpring model is a deformable model (a set of masses connected by springs).
/// Note that the state is stored into a TetrahedronMesh for the sake of re usability.
///  -> The masses are specific data for each vertex (position, velocity and mass).
///  -> The linear springs are specific data for each edge (stiffness, damping).
/// Note that internal calculation is done with Eigen data structure for the sake of efficiency
/// The class can handle 3 type of numerical integration scheme (Euler explicit, modified and implicit).
/// The model handles damping through the Rayleigh damping (damping is a combination of mass and stiffness).
/// Boundary conditions can be defined on the model, which are the fixed node ids.
class MassSpringRepresentation: public Representation
{
public:
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign> Vector;

	/// The diverse numerical integration scheme supported
	enum IntegrationScheme {
		INTEGRATIONSCHEME_EXPLICIT_EULER = 0,
		INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
		INTEGRATIONSCHEME_IMPLICIT_EULER
	};

	/// Constructor
	/// \param name The name of the MassSpringRepresentation
	explicit MassSpringRepresentation(const std::string& name);

	/// Destructor
	virtual ~MassSpringRepresentation();

	/// Gets the number of masses
	/// \return the number of masses
	unsigned int getNumMasses() const;
	/// Gets the number of springs
	/// \return the number of springs
	unsigned int getNumSprings() const;

	/// Retrieves the mass of a given node
	/// \param nodeId The node id for which the mass is requested
	/// \return the mass attribute of a node
	MassParameter& getMassParameter(unsigned int nodeId);
	/// Retrieves a given spring from its id
	/// \param springId The spring id for which the spring is requested
	/// \return the spring for the given springId
	LinearSpringParameter& getSpringParameter(unsigned int springId);

	/// Gets the current state (as a TetrahedronMesh<MassParameter, LinearSpringParameter, void, void)
	/// \return the current state
	const TetrahedronMesh<MassParameter, LinearSpringParameter, void, void>& getFinalState() const;
	/// Gets the initial state (as a TetrahedronMesh<MassParameter, LinearSpringParameter, void, void)
	/// \return the current state
	const TetrahedronMesh<MassParameter, LinearSpringParameter, void, void>& getInitialState() const;

	/// Gets the total mass of the mass spring
	/// \return The total mass of the mass spring (in Kg)
	double getTotalMass() const;

	/// Gets the Rayleigh stiffness parameter
	/// \return The Rayleigh stiffness parameter
	double getRayleighDampingStiffness() const;
	/// Gets the Rayleigh mass parameter
	/// \return The Rayleigh mass parameter
	double getRayleighDampingMass() const;
	/// Sets the Rayleigh stiffness parameter
	/// \param stiffnessCoef The Rayleigh stiffness parameter
	void setRayleighDampingStiffness(double stiffnessCoef);
	/// Sets the Rayleigh mass  parameter
	/// \param massCoef The Rayleigh mass parameter
	void setRayleighDampingMass(double massCoef);

	/// Adds a boundary condition to the mass spring
	/// \param nodeId The id of the node to fix
	/// \note
	void addBoundaryCondition(unsigned int nodeId);
	/// Gets a specific boundary condition
	/// \param bcId The id of the boundary condition to retrieve
	/// \return The requested boundary condition (i.e. a node id) or throw an exception is bcId is invalid
	unsigned int getBoundaryCondition(unsigned int boundaryConditionId) const;
	/// Gets the number of boundary conditions
	/// \return The number of boundary conditions
	unsigned int getNumBoundaryConditions() const;

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	void setIntegrationScheme(IntegrationScheme integrationScheme);
	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	IntegrationScheme getIntegrationScheme() const;

	/// Initializes a 1D model
	/// \param extremities Array of 2 positions forming the extremities of the 1D model
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 1)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param springStiffness The spring stiffness for all springs
	/// \param springDamping The spring damping for all springs
	void init1D(const Vector3d extremities[2], unsigned int numNodesPerDim[1],
		double totalMass, double springStiffness, double springDamping);
	/// Initializes a 2D model
	/// \param extremities 4 positions forming the extremities of the 2D regular model (4 corners)
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 2)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param springStiffness The spring stiffness for all springs
	/// \param springDamping The spring damping for all springs
	void init2D(const Vector3d extremities[2][2], unsigned int numNodesPerDim[2],
		double totalMass, double springStiffness, double springDamping);
	/// Initializes a 3D model
	/// \param extremities 8 positions forming the extremities of the 3D regular model (8 corners)
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 3)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param springStiffness The spring stiffness for all springs
	/// \param springDamping The spring damping for all springs
	void init3D(const Vector3d extremities[2][2][2], unsigned int numNodesPerDim[3],
		double totalMass, double springStiffness, double springDamping);

	/// Query the representation type
	/// \return the RepresentationType for this representation
	virtual RepresentationType getType() const override;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt) override;

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt) override;

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt) override;

	/// Apply a correction to the internal degrees of freedom
	/// \param dt The time step
	/// \param block The block of a vector containing the correction to be applied to the dof
	virtual void applyDofCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::MlcpSolution::Vector>& block) override;

	/// Set the initial pose of the representation
	/// \param pose The initial pose
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) override
	{
		m_initialPose = pose;
	}

	/// Get the initial pose of the representation
	/// \return The initial pose (always identity)
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const override
	{
		return m_initialPose;
	}

	/// Set the pose of the representation
	/// \param pose The pose to set the representation to
	/// \note Impossible for a MassSpring (always in global space => pose = identity)
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) override
	{
		SURGSIM_ASSERT(false) << "Cannot set the pose of a MassSpring";
	}

	/// Get the pose of the representation
	/// \return The pose of this representation (always Identity)
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override
	{
		return m_identityPose;
	}

protected:
	/// Allocate all the data structure for a given size
	/// \param numDof The number of Dof to account for in all the data structure
	void allocate(int numDof);

	/// Update method for the Euler explicit numerical integration scheme
	/// \param dt The time step
	/// \param useModifiedEuler True if modified Euler should be used, False if not (default = false)
	/// \note The modified Euler explicit is a semi-implicit scheme, therefore slightly more stable than explicit
	void updateEulerExplicit(double dt, bool useModifiedEuler = false);

	/// Add the Rayleigh damping forces to f
	/// \param[in,out] f The force vector to cumulate the Rayleigh damping force into
	/// \param v The velocity vector
	/// \param scale A scaling factor to apply on the damping force
	/// \note M.a + D.v + K.x = F          with D = c.M + d.K (Rayleigh damping definition)
	/// \note M.a + K.x = F - (c.M.v + d.K.v)
	void addRayleighDampingForce(Vector* f, const Vector& v, double scale = 1.0);

	/// Add the spring forces to f (given the state x,v)
	/// \param[in,out] f The force vector to cumulate the spring forces into
	/// \param x, v the position and velocity state vector
	/// \param scale A scaling factor to scale the spring forces with
	void addSpringForces(Vector* f, const Vector& x, const Vector& v, double scale = 1.0);

	/// Internal Eigen vectors to store/compute current/previous position
	Vector m_x, m_xPrevious;
	/// Internal Eigen vectors to store/compute current velocity
	Vector m_v;
	/// Internal Eigen vectors to store/compute current acceleration
	Vector m_a;
	/// Internal Eigen vectors to store/compute current force
	Vector m_f;

private:
	/// Initial and final states (stored as TetrahedronMeshes)
	MassSpringRepresentationState m_initialState, m_finalState;

	/// Identity pose: stored positions are always in global space (Identity transformation)
	SurgSim::Math::RigidTransform3d m_identityPose;
	/// Initial pose that will be used to transform the mesh on initialization
	/// Stored positions are always in global space (Identity transformation)
	SurgSim::Math::RigidTransform3d m_initialPose;

	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, Mass = mass, K = stiffness
	struct {
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;

	/// Boundary conditions (Fixed node ids)
	std::set<unsigned int> m_boundaryConditions;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	IntegrationScheme m_integrationScheme;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H