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

#ifndef SURGSIM_PHYSICS_FEM2DELEMENTTRIANGLE_H
#define SURGSIM_PHYSICS_FEM2DELEMENTTRIANGLE_H

#include <array>

#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

/// 2D FemElement based on a triangle with a constant thickness
///
/// The triangle is modelled as a shell (6DOF) which is decomposed into a membrane (in-plane 2DOF (X,Y)) and
/// a plate (bending/twisting 3DOF (Z, ThetaX,ThetaY)). The thin-plate assumption does not consider the drilling
/// dof (ThetaZ). The system includes the DOF for completeness but does not assign any mass or stiffness to it.
///
/// The membrane (in-plane) equations (mass and stiffness) are following
/// "Theory of Matrix Structural Analysis" from J.S. Przemieniecki.
///
/// The thin-plate (bending) equations (mass and stiffness) are following
/// "A Study Of Three-Node Triangular Plate Bending Elements", Jean-Louis Batoz
/// Numerical Methods in Engineering, vol 15, 1771-1812 (1980)
/// \note The plate mass matrix is not detailed in the above paper, but the analytical equations
/// \note have been derived from it.
///
/// \note The element is considered to have a constant thickness.
class Fem2DElementTriangle : public FemElement
{
	typedef Eigen::Matrix<double, 3, 3> Matrix33Type;

	typedef Eigen::Matrix<double, 3, 6> Matrix36Type;
	typedef Eigen::Matrix<double, 6, 6> Matrix66Type;

	typedef Eigen::Matrix<double, 3, 9> Matrix39Type;
	typedef Eigen::Matrix<double, 9, 9> Matrix99Type;

public:
	/// Constructor
	/// \param nodeIds An array of 3 node ids (A, B, C) defining this triangle element with respect to a
	/// DeformableRepresentaitonState which is passed to the initialize method.
	explicit Fem2DElementTriangle(std::array<size_t, 3> nodeIds);

	/// Sets the triangle's thickness
	/// \param thickness The thickness of the triangle
	void setThickness(double thickness);

	/// Gets the triangle's thickness
	/// \return The thickness of the triangle
	double getThickness() const;

	/// Initializes the FemElement once everything has been set
	/// \param state The state to initialize the FemElement with
	/// \note We use the theory of linear elasticity, so this method pre-computes the stiffness and mass matrices
	virtual void initialize(const SurgSim::Math::OdeState& state) override;

	/// Gets the element's volume based on the input state
	/// \param state The state to compute the volume with
	/// \return The element's volume
	virtual double getVolume(const SurgSim::Math::OdeState& state) const override;

	/// Adds the element's force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the element's force into
	/// \param scale A factor to scale the added force with
	/// \note The element's force is of size (getNumDofPerNode() x getNumNodes()).
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	virtual void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
						  double scale = 1.0) override;

	/// Adds the element's mass matrix M (computed for a given state) to a complete system mass matrix M (assembly)
	/// \param state The state to compute the mass matrix with
	/// \param[in,out] M The complete system mass matrix to add the element's mass-matrix into
	/// \param scale A factor to scale the added mass matrix with
	/// \note The element's mass matrix is a square matrix of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	virtual void addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* M,
						 double scale = 1.0) override;

	/// Adds the element's damping matrix D (= -df/dv) (computed for a given state) to a complete system damping matrix
	/// D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	/// \note The element's damping matrix is a square matrix of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	/// \note The beam uses linear elasticity (not visco-elasticity), so it does not have any damping.
	virtual void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* D,
							double scale = 1.0) override;

	/// Adds the element's stiffness matrix K (= -df/dx) (computed for a given state) to a complete system stiffness
	/// matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	virtual void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K,
							  double scale = 1.0) override;

	/// Adds the element's force vector, mass, stiffness and damping matrices (computed for a given state) into a
	/// complete system data structure F, M, D, K (assembly)
	/// \param state The state to compute everything with
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param[in,out] M The complete system mass matrix to add the element mass matrix into
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	virtual void addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, SurgSim::Math::Matrix* M,
						 SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K) override;

	/// Adds the element's matrix-vector contribution F += (alphaM.M + alphaD.D + alphaK.K).x (computed for a given
	/// state) into a complete system data structure F (assembly)
	/// \param state The state to compute everything with
	/// \param alphaM The scaling factor for the mass contribution
	/// \param alphaD The scaling factor for the damping contribution
	/// \param alphaK The scaling factor for the stiffness contribution
	/// \param x A complete system vector to use as the vector in the matrix-vector multiplication
	/// \param[in,out] F The complete system force vector to add the element matrix-vector contribution into
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	virtual void addMatVec(const SurgSim::Math::OdeState& state, double alphaM, double alphaD, double alphaK,
						   const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F);

	virtual SurgSim::Math::Vector computeCartesianCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& naturalCoordinate) const;

	virtual SurgSim::Math::Vector computeNaturalCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& cartesianCoordinate) const override;

protected:
	/// Computes the triangle element's rotation given a state
	/// \param state The state to compute the rotation from
	/// \return The rotation matrix of the element in the given state
	SurgSim::Math::Matrix33d computeRotation(const SurgSim::Math::OdeState& state);

	/// Computes the triangle's stiffness matrix
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] k The stiffness matrix to store the result into
	void computeStiffness(const SurgSim::Math::OdeState& state,
		Eigen::Matrix<double, 18, 18>* k);

	/// Computes the triangle's mass matrix
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const SurgSim::Math::OdeState& state, Eigen::Matrix<double, 18, 18>* m);

	/// The element's rest state
	Eigen::Matrix<double, 18, 1> m_x0;

	/// Initial rotation matrix for the element
	SurgSim::Math::Matrix33d m_initialRotation;

	/// Mass matrix (in global coordinate frame)
	Eigen::Matrix<double, 18, 18> m_M;
	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 18, 18> m_MLocal;
	/// Stiffness matrix (in global coordinate frame)
	Eigen::Matrix<double, 18, 18> m_K;
	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 18, 18> m_KLocal;

	/// The triangle rest area
	double m_restArea;

	/// Thickness of the element
	double m_thickness;

	/// Compute the various shape functions (membrane and plate deformations) parameters
	/// \param restState the rest state to compute the shape functions paramters from
	void computeShapeFunctionsParameters(const SurgSim::Math::OdeState& restState);

	/// Membrane (in-plane) deformation. DOF simulated: (x, y)
	/// "Theory of Matrix Structural Analysis" from J.S. Przemieniecki
	/// Shape functions fi(x, y) = ai + bi.x + ci.y
	SurgSim::Math::Matrix33d m_membraneShapeFunctionsParameters; //< Stores (ai, bi, ci) on each row

	/// Thin-plate (bending/twisting) specific data structure
	/// DOF simulated: (z, thetaX, thetaY)
	/// "A Study Of Three-Node Triangular Plate Bending Elements", Jean-Louis Batoz
	/// Numerical Methods in Engineering, vol 15, 1771-1812 (1980)
	/// Indices are as follow:
	/// 0 1 2 denotes triangle's points ABC:
	/// 4 (mid-edge 12) 5 (mid-edge 20) 6 (mid-edge 01) denotes mid-edge points
	/// Data structures having only mid-edge information are 0 based (0->4 (mid-egde 12) ; 1->5 ; 2->6)
	SurgSim::Math::Vector3d m_xij;     //< xi - xj
	SurgSim::Math::Vector3d m_yij;     //< yi - yj
	SurgSim::Math::Vector3d m_lij_sqr; //< xij^2 + yij^2
	SurgSim::Math::Vector3d m_ak;      //< -xij/li^2
	SurgSim::Math::Vector3d m_bk;      //< 3/4 xij yij/lij2
	SurgSim::Math::Vector3d m_ck;      //< (1/4 xij^2 - 1/2 yij^2)/lij^2
	SurgSim::Math::Vector3d m_dk;      //< -yij/lij^2
	SurgSim::Math::Vector3d m_ek;      //< (1/4 yij^2 - 1/2 xij^2)/lij^2
	//...and more variables for the derivatives
	SurgSim::Math::Vector3d m_Pk;      //< -6xij/lij^2    = 6 m_ak
	SurgSim::Math::Vector3d m_qk;      //< 3xijyij/lij^2  = 4 m_bk
	SurgSim::Math::Vector3d m_tk;      //< -6yij/lij^2    = 6 m_dk
	SurgSim::Math::Vector3d m_rk;      //< 3yij^2/lij^2
	/// Batoz derivative dHx/dxi
	/// \param xi, neta The parametric coordinate (in [0 1] and xi+neta<1.0)
	/// \return The vector dHx/dxi evaluated at (xi, neta)
	std::array<double, 9> batozDhxDxi(double xi, double neta) const;
	/// Batoz derivative dHx/dneta
	/// \param xi, neta The parametric coordinate (in [0 1] and xi+neta<1.0)
	/// \return The vector dHx/dneta evaluated at (xi, neta)
	std::array<double, 9> batozDhxDneta(double xi, double neta) const;
	/// Batoz derivative dHy/dxi
	/// \param xi, neta The parametric coordinate (in [0 1] and xi+neta<1.0)
	/// \return The vector dHy/dxi evaluated at (xi, neta)
	std::array<double, 9> batozDhyDxi(double xi, double neta) const;
	/// Batoz derivative dHy/dneta
	/// \param xi, neta The parametric coordinate (in [0 1] and xi+neta<1.0)
	/// \return The vector dHy/dneta evaluated at (xi, neta)
	std::array<double, 9> batozDhyDneta(double xi, double neta) const;
	/// Batoz strain displacement matrix evaluated at a given point
	/// \param xi, neta The parametric coordinate (in [0 1] and xi+neta<1.0)
	/// \return The 3x9 strain displacement matrix evaluated at (xi, neta)
	Matrix39Type batozStrainDisplacement(double xi, double neta) const;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM2DELEMENTTRIANGLE_H
