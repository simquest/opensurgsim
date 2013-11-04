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

	/// Computes the element force from a given state
	/// \param state The state to compute the force with
	/// \return The computed element force of size (getNumDofPerNode() x getNumNodes())
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	virtual const SurgSim::Math::Vector& computeForce(const DeformableRepresentationState& state) override;

	/// Computes the element mass matrix M from a given state
	/// \param state The state to compute the mass matrix with
	/// \return The computed element square mass matrix  of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Matrix& computeMass(const DeformableRepresentationState& state) override;

	/// Computes the element damping matrix D (= -df/dv) from a given state
	/// \param state The state to compute the damping matrix with
	/// \return The computed element square damping matrix  of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Matrix& computeDamping(const DeformableRepresentationState& state) override;

	/// Computes the element stiffness matrix K (= -df/dx) from a given state
	/// \param state The state to compute the stiffness matrix with
	/// \return The computed element square stiffness matrix of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses an internal data member to store and return the result
	/// \note This method suppose that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual const SurgSim::Math::Matrix& computeStiffness(const DeformableRepresentationState& state) override;

	/// Computes the element force vector, mass, stiffness and damping matrices from a given state
	/// \param state The state to compute everything with
	/// \param[out] f The computed element force of size (getNumDofPerNode() x getNumNodes())
	/// \param[out] M The computed element square mass of size getNumDofPerNode() x getNumNodes()
	/// \param[out] D The computed element square damping matrix of size getNumDofPerNode() x getNumNodes()
	/// \param[out] K The computed element square stiffness matrix of size getNumDofPerNode() x getNumNodes()
	/// \note (non-const) because it uses internal data members to store and return the results
	/// \note This method suppose that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	virtual void computeFMDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector** f,
		SurgSim::Math::Matrix** M, SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K) override;

protected:
	/// Computes the determinant of 3 vectors
	/// \param a, b, c The 3 vectors to compute the determinant from
	/// \return |a b c|, The determinant of the 3 vectors a, b and c
	double det(const Vector3d& a, const Vector3d& b, const Vector3d& c) const;

	/// Computes the tetrahdron shape functions
	/// \param restState The deformable rest state to compute the shape function from
	void computeShapeFunctions(const DeformableRepresentationState& restState);

	/// Mass density (in Kg.m-3)
	double m_rho;
	/// Young modulus (in N.m-2)
	double m_E;
	/// Poisson ratio (unitless)
	double m_nu;

	/// Tetrahedron base functions coefficients Ni(x,y,z) = 1/6V ( ai + x.bi + y.ci + z.di )
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
