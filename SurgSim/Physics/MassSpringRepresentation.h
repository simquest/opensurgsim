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

#include <SurgSim/Physics/DeformableRepresentation.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>
#include <SurgSim/DataStructures/TetrahedronMesh.h>
#include <Surgsim/Math/Vector.h>
#include <Surgsim/Math/Matrix.h>

using SurgSim::DataStructures::TetrahedronMesh;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;

namespace SurgSim
{

namespace Physics
{

class Mass
{
public:
	Mass(double m = 0.0) : m_mass(m){}

	void setMass(double mass){ m_mass = mass; }
	double getMass() const { return m_mass; }

	bool operator ==(const Mass& m) const { return (m_mass == m.m_mass); }
	bool operator !=(const Mass& m) const { return !((*this) == m); }

protected:
	double m_mass;
};

class LinearSpring
{
public:

	void setStiffness(double k){}
	void setDamping(double d){}
	const Vector3d& getF() const { return m_F; }
	const Matrix33d& getdF_dx() const { return m_dF_dx; }
	const Matrix33d& getdF_dv() const { return m_dF_dv; }

	void update(const Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>& x,
		const Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>& v) {}

	bool operator ==(const LinearSpring& m) const { return true; }
	bool operator !=(const LinearSpring& m) const { return !((*this) == m); }

private:
	Vector3d m_F;
	Matrix33d m_dF_dx, m_dF_dv;
};

/// MassSpring model is a specialized deformable model (a set of masses connected by springs).
/// Note that the structure is stored into a TetrahedronMesh for the sake of reusability.
///  -> The masses are specific data for each vertex.
///  -> The linear springs are specific data for each edge.
/// The class can handle 3 type of numerical integration scheme (Euler explicit, modified and implicit).
/// The model handles damping through the Rayleigh damping (damping is a combination of mass and stiffness).
/// Boundary conditions can be defined on the model, which are the fixed node ids.
class MassSpringRepresentation: public DeformableRepresentation
{
public:
	typedef Eigen::Matrix<double, Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

	/// The diverse numerical integration scheme supported
	enum IntegrationScheme {
		INTEGRATIONSCHEME_EXPLICIT_EULER = 0,
		INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
		INTEGRATIONSCHEME_IMPLICIT_EULER
	};

	/// Constructor
	/// \param name The name of the MassSpringRepresentation
	MassSpringRepresentation(const std::string& name);

	/// Destructor
	virtual ~MassSpringRepresentation();

	/// Gets the number of masses
	/// \return the number of masses
	unsigned int getNumMasses(void) const;
	/// Gets the number of springs
	/// \return the number of springs
	unsigned int getNumSprings(void) const;
	/// Retrieves the mass of a given node
	/// \param nodeId The node id for which the mass is requested
	/// \return the mass attribute of a node
	const Mass& getMass(unsigned int nodeId) const;
	/// Retrieves a given spring from its id
	/// \param springId The spring id for which the spring is requested
	/// \return the spring for the given springId
	const LinearSpring& getSpring(unsigned int springId) const;

	/// Gets the total mass of the mass spring
	/// \return The total mass of the mass spring (in Kg)
	double getTotalMass(void) const;

	/// Gets the Rayleigh stiffness parameter
	/// \return The Rayleigh stiffness parameter
	double getRayleighDampingStiffness(void) const;
	/// Gets the Rayleigh mass parameter
	/// \return The Rayleigh mass parameter
	double getRayleighDampingMass(void) const;
	/// Sets the Rayleigh stiffness parameter
	/// \param stiffnessCoef The Rayleigh stiffness parameter
	void setRayleighDampingStiffness(double stiffnessCoef);
	/// Sets the Rayleigh mass  parameter
	/// \param massCoef The Rayleigh mass parameter
	void setRayleighDampingMass(double massCoef);

	/// Adds a boundary condition to the mass spring
	/// \param nodeId The id of the node to fix
	void addBoundaryCondition(int nodeId);
	/// Gets a specific boundary condition
	/// \param bcId The id of the boundary condition to retrieve
	/// \return The requested boundary condition (i.e. a node id)
	int getBoundaryCondition(size_t bcId) const;
	/// Gets the number of boundary conditions
	/// \return The number of boundary conditions
	size_t getNumBoundaryConditions(void) const;

	/// Sets the numerical integration scheme
	/// \param integrationScheme The integration scheme to use
	void setIntegrationScheme(IntegrationScheme integrationScheme);
	/// Gets the numerical integration scheme
	/// \return The integration scheme currently in use
	IntegrationScheme getIntegrationScheme(void) const;

	/// Initializes a 1D model
	/// \param extremities Array of 2 positions forming the extremities of the 1D model
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 1)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param springStiffness The spring stiffness for all springs
	/// \param springDamping The spring damping for all springs
	void init1D(const Vector3d extremities[2], int numNodesPerDim[1],
		double totalMass, double springStiffness, double springDamping);
	/// Initializes a 2D model
	/// \param extremities 4 positions forming the extremities of the 2D regular model (4 corners)
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 2)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param springStiffness The spring stiffness for all springs
	/// \param springDamping The spring damping for all springs
	void init2D(const Vector3d extremities[2][2], int numNodesPerDim[2],
		double totalMass, double springStiffness, double springDamping);
	/// Initializes a 3D model
	/// \param extremities 8 positions forming the extremities of the 3D regular model (8 corners)
	/// \param numNodesPerDim The number of nodes to be created for each dimension (here 3)
	/// \param totalMass The total mass of the mass spring (evenly spread out on the masses)
	/// \param springStiffness The spring stiffness for all springs
	/// \param springDamping The spring damping for all springs
	void init3D(const Vector3d extremities[2][2][2], int numNodesPerDim[3],
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
	virtual void applyDofCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::MlcpSolution::Vector>& block);

protected:
	/// Allocate all the data structure for a given size
	/// \param numDof The number of Dof to account for in all the data structure
	void allocate(int numDof);

	/// Update method for the Euler explicit numerical integration scheme
	/// \param dt The time step
	/// \param useModifiedEuler True if modified Euler should be used, False if not (default = false)
	/// \note The modified Euler explicit is a semi-implicit scheme, therefore slightly more stable than explicit
	void updateEulerExplicit(double dt, bool useModifiedEuler = false);

private:
	/// TetrahedronMesh containing the Masses (on the vertices) and Springs (on the edges)
	TetrahedronMesh<Mass, LinearSpring, void, void> m_tetrahedronMesh;

	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, Mass = mass, K = stiffness
	struct {
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;

	/// Boundary conditions (Fixed node ids)
	std::vector<int> m_boundaryConditions;

	/// Numerical Integration scheme (dynamic explicit/implicit solver)
	IntegrationScheme m_integrationScheme;

	/// Current forces applied on each dof
	Vector m_f;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H