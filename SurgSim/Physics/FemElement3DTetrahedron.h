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

#ifndef SURGSIM_PHYSICS_FEMELEMENT3DTETRAHEDRON_H
#define SURGSIM_PHYSICS_FEMELEMENT3DTETRAHEDRON_H

#include <array>
#include <SurgSim/Physics/FemElement.h>

using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

/// Class for Fem Element 3D based on a tetrahedron volume discretization
/// \note The inertia property (mass) of the tetrahedron is derived from
/// \note    "Theory of Matrix Structural Analysis" from J.S. Przemieniecki
/// \note The force and stiffness matrix of the tetrahedron is derived from
/// \note    http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
/// \note The deformation is based on the elasticity theory and not on the visco-elasticity theory.
/// \note Therefore the element does not have any damping component.
class FemElement3DTetrahedron : public FemElement
{
public:
	/// Constructor
	/// \param nodeIds An array of 4 node ids defining this tetrahedron element in a overall mesh
	/// \param restState The rest state to initialize the Tetrahedron with
	FemElement3DTetrahedron(std::array<unsigned int, 4> nodeIds, const DeformableRepresentationState& restState);

	/// Sets the mass density (in Kg.m-3)
	/// \param rho The mass density
	void setMassDensity(double rho);
	/// Gets the mass density (in Kg.m-3)
	/// \return The mass density
	double getMassDensity() const;

	/// Sets the Young modulus (in N.m-2)
	/// \param E The Young modulus
	void setYoungModulus(double E);
	/// Gets the Young modulus (in N.m-2)
	/// \return The Young modulus
	double getYoungModulus() const;

	/// Sets the Poisson ratio (unitless)
	/// \param nu The Poisson ratio
	void setPoissonRatio(double nu);
	/// Gets the Poisson ratio (unitless)
	/// \return The Poisson ratio
	double getPoissonRatio() const;

	/// Get the element volume based on the input state
	/// \param state The deformable state to compute the volume with
	virtual double getVolume(const DeformableRepresentationState& state) const override;

	/// Adds the element force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \note The element force is of size (getNumDofPerNode() x getNumNodes())
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	virtual void addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F) override;

	/// Adds the element mass matrix M (computed for a given state) to a complete system mass matrix M (assembly)
	/// \param state The state to compute the mass matrix with
	/// \param[in,out] M The complete system mass matrix to add the element mass-matrix into
	/// \note The element mass matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual void addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M) override;

	/// Adds the element damping matrix D (= -df/dv) (comuted for a given state)
	/// to a complete system damping matrix D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \note The element damping matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual void addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D) override;

	/// Adds the element stiffness matrix K (= -df/dx) (computed for a given state)
	/// to a complete system stiffness matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual void addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K) override;

	/// Adds the element force vector, mass, stiffness and damping matrices (computed for a given state)
	/// into a complete system data structure F, M, D, K (assembly)
	/// \param state The state to compute everything with
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param[in,out] M The complete system mass matrix to add the element mass matrix into
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	virtual void addFMDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector* F,
		SurgSim::Math::Matrix* M,
		SurgSim::Math::Matrix* D,
		SurgSim::Math::Matrix* K) override;

protected:
	/// Computes the determinant of 3 vectors
	/// \param a, b, c The 3 vectors to compute the determinant from
	/// \return |a b c|, The determinant of the 3 vectors a, b and c
	double det(const Vector3d& a, const Vector3d& b, const Vector3d& c) const;

	/// Computes the tetrahdron shape functions
	/// \param restState The deformable rest state to compute the shape function from
	void computeShapeFunctions(const DeformableRepresentationState& restState);

	/// Computes the tetrahedron stiffness matrix
	/// \param state The deformable state to compute the stiffness matrix from
	/// \param[out] k The stiffness matrix to store the result into
	void computeStiffness(const DeformableRepresentationState& state, Eigen::Matrix<double, 12, 12>* k);

	/// Mass density (in Kg.m-3)
	double m_rho;
	/// Young modulus (in N.m-2)
	double m_E;
	/// Poisson ratio (unitless)
	double m_nu;

	/// Tetrahedron shape functions coefficients Ni(x,y,z) = 1/6V ( ai + x.bi + y.ci + z.di )
	double m_restVolume, m_ai[4], m_bi[4], m_ci[4], m_di[4];

	/// The tetrahedon rest state
	Eigen::Matrix<double, 12, 1, Eigen::DontAlign> m_x0;

	/// Elasticity material matrix (contains the elastic properties of the material)
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> m_Em;
	/// Strain matrix
	Eigen::Matrix<double, 6, 12, Eigen::DontAlign> m_strain;
	/// Stress matrix
	Eigen::Matrix<double, 6, 12, Eigen::DontAlign> m_stress;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENT3DTETRAHEDRON_H
