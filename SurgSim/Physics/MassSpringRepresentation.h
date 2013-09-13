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

	bool operator ==(const LinearSpring& m) const { return true; }
	bool operator !=(const LinearSpring& m) const { return !((*this) == m); }

private:
	Vector3d m_F;
	Matrix33d m_dF_dx, m_dF_dv;
};

/// MassSpring model
class MassSpringRepresentation: public DeformableRepresentation
{
public:
	typedef Eigen::Matrix<double, Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

	enum IntegrationScheme {
		INTEGRATIONSCHEME_EXPLICIT_EULER = 0,
		INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
		INTEGRATIONSCHEME_IMPLICIT_EULER
	};

	MassSpringRepresentation(const std::string& name);
	virtual ~MassSpringRepresentation();

	unsigned int getNumMasses(void) const;
	unsigned int getNumSprings(void) const;
	const Mass& getMass(unsigned int massId) const;
	const LinearSpring& getSpring(unsigned int springId) const;

	double getTotalMass(void) const;

	double getRayleighDampingStiffness(void) const;
	double getRayleighDampingMass(void) const;
	void setRayleighDampingStiffness(double stiffnessCoef);
	void setRayleighDampingMass(double massCoef);

	void addBC(int nodeID);
	int getBC(size_t bcID) const;
	size_t getNumBC(void) const;

	void setIntegrationScheme(IntegrationScheme integrationScheme);
	IntegrationScheme getIntegrationScheme(void) const;

	//! Initialization
	void init1D(const Vector3d extremities[2], int numNodesPerDim[1], double totalMass, double springStiffness, double springDamping);
	void init2D(const Vector3d extremities[2][2], int numNodesPerDim[2]);
	void init3D(const Vector3d extremities[2][2][2], int numNodesPerDim[3]);

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
	void allocate(int numDof);

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

	///// Initial, current and previous dof position
	//Vector m_x0, m_x, m_xprev;

	///// Initial and current dof velocity
	//Vector m_v0, m_v;

	/// Current forces applied on each dof
	Vector m_f;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H