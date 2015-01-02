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
/// The triangle is modeled as a shell (6DOF) which is decomposed into a membrane (in-plane 2DOF \f$(x, y)\f$) and a
/// plate (bending/twisting 3DOF \f$(z, \theta_x, \theta_y)\f$). The thin-plate assumption does not consider the
/// drilling dof \f$\theta_z\f$.
/// The system includes the drilling DOF for completeness but does not assign any mass or stiffness to it.
///
/// The membrane (in-plane) equations (mass and stiffness) are following
/// "Theory of Matrix Structural Analysis" from J.S. Przemieniecki.
///
/// The thin-plate (bending) equations (mass and stiffness) are following
/// "A Study Of Three-Node Triangular Plate Bending Elements", Jean-Louis Batoz
/// Numerical Methods in Engineering, vol 15, 1771-1812 (1980). <br>
/// The plate mass matrix is not detailed in the above paper, but the analytical equations
/// have been derived from it. Moreover, to account for contribution of the displacement
/// along z to the plate mass matrix, we used a cubic expression of this displacement given in:
/// "Shell elements: modelizations DKT, DST, DKTG and Q4g", Code_Aster, 2013, Thomas De Soza.
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
	void initialize(const SurgSim::Math::OdeState& state) override;

	/// Gets the element's volume based on the input state
	/// \param state The state to compute the volume with
	/// \return The element's volume
	double getVolume(const SurgSim::Math::OdeState& state) const override;

	/// Adds the element's force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the element's force into
	/// \param scale A factor to scale the added force with
	/// \note The element's force is of size (getNumDofPerNode() x getNumNodes()).
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale = 1.0) override;

	/// Adds the element's mass matrix M (computed for a given state) to a complete system mass matrix M (assembly)
	/// \param state The state to compute the mass matrix with
	/// \param[in,out] M The complete system mass matrix to add the element's mass-matrix into
	/// \param scale A factor to scale the added mass matrix with
	/// \note The element's mass matrix is a square matrix of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	void addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* M, double scale = 1.0) override;

	/// Adds the element's damping matrix D (= -df/dv) (computed for a given state) to a complete system damping matrix
	/// D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	/// \note The element's damping matrix is a square matrix of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	/// \note The beam uses linear elasticity (not visco-elasticity), so it does not have any damping.
	void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* D, double scale = 1.0) override;

	/// Adds the element's stiffness matrix K (= -df/dx) (computed for a given state) to a complete system stiffness
	/// matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K, double scale = 1.0) override;

	/// Adds the element's force vector, mass, stiffness and damping matrices (computed for a given state) into a
	/// complete system data structure F, M, D, K (assembly)
	/// \param state The state to compute everything with
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param[in,out] M The complete system mass matrix to add the element mass matrix into
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	void addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, SurgSim::Math::Matrix* M,
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

	virtual SurgSim::Math::Vector computeCartesianCoordinate(const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& naturalCoordinate) const;

	SurgSim::Math::Vector computeNaturalCoordinate(const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& cartesianCoordinate) const override;

protected:
	/// Computes the triangle element's rotation given a state
	/// \param state The state to compute the rotation from
	/// \return The rotation matrix of the element in the given state
	SurgSim::Math::Matrix33d computeRotation(const SurgSim::Math::OdeState& state);

	/// Computes the triangle's local stiffness matrix
	/// \param state The state to compute the local stiffness matrix from
	/// \param[out] localStiffnessMatrix The local stiffness matrix to store the result into
	virtual void computeLocalStiffness(const SurgSim::Math::OdeState& state,
									   Eigen::Matrix<double, 18, 18>* localStiffnessMatrix);

	/// Computes the triangle's stiffness matrix
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] stiffnessMatrix The stiffness matrix to store the result into
	void computeStiffness(const SurgSim::Math::OdeState& state, Eigen::Matrix<double, 18, 18>* stiffnessMatrix);

	/// Computes the triangle's local mass matrix
	/// \param state The state to compute the local mass matrix from
	/// \param[out] localMassMatrix The local mass matrix to store the result into
	virtual void computeLocalMass(const SurgSim::Math::OdeState& state,
								  Eigen::Matrix<double, 18, 18>* localMassMatrix);

	/// Computes the triangle's mass matrix
	/// \param state The state to compute the mass matrix from
	/// \param[out] massMatrix The mass matrix to store the result into
	void computeMass(const SurgSim::Math::OdeState& state, Eigen::Matrix<double, 18, 18>* massMatrix);

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
	/// \param restState the rest state to compute the shape functions parameters from
	void computeShapeFunctionsParameters(const SurgSim::Math::OdeState& restState);

	/// Membrane (in-plane) deformation. DOF simulated: (x, y).
	/// "Theory of Matrix Structural Analysis" from J.S. Przemieniecki.
	/// Shape functions are \f$f_i(x, y) = a_i + b_i.x + c_i.y\f$, here we store \f$(a_i, b_i, c_i)\f$ on each row.
	SurgSim::Math::Matrix33d m_membraneShapeFunctionsParameters;

	// Thin-plate (bending/twisting) specific data structure
	// DOF simulated: (z, thetaX, thetaY)
	// "A Study Of Three-Node Triangular Plate Bending Elements", Jean-Louis Batoz
	// Numerical Methods in Engineering, vol 15, 1771-1812 (1980)
	// Indices are as follow:
	// 0 1 2 denotes triangle's points ABC:
	// 4 (mid-edge 12) 5 (mid-edge 20) 6 (mid-edge 01) denotes mid-edge points
	// Data structures having only mid-edge information are 0 based (0->4 (mid-egde 12) ; 1->5 ; 2->6)

	//@{
	SurgSim::Math::Vector3d m_xij;     ///< Batoz variable \f$x_{ij} = x_i - x_j\f$
	SurgSim::Math::Vector3d m_yij;     ///< Batoz variable \f$y_{ij} = y_i - y_j\f$
	SurgSim::Math::Vector3d m_lij_sqr; ///< Batoz variable \f$l_{ij}^2 = x_{ij}^2 + y_{ij}^2\f$
	SurgSim::Math::Vector3d m_ak;      ///< Batoz variable \f$a_{k} = -x_{ij}/l_i^2\f$
	SurgSim::Math::Vector3d m_bk;      ///< Batoz variable \f$b_{k} = 3/4 x_{ij} y_{ij}/l_{ij}2\f$
	SurgSim::Math::Vector3d m_ck;      ///< Batoz variable \f$c_{k} = (1/4 x_{ij}^2 - 1/2 y_{ij}^2)/l_{ij}^2\f$
	SurgSim::Math::Vector3d m_dk;      ///< Batoz variable \f$d_{k} = -y_{ij}/l_{ij}^2\f$
	SurgSim::Math::Vector3d m_ek;      ///< Batoz variable \f$e_{k} = (1/4 y_{ij}^2 - 1/2 x_{ij}^2)/l_{ij}^2\f$
	//@}

	//@{
	SurgSim::Math::Vector3d m_Pk;      ///< Batoz variable \f$P_{k} = -6x_{ij}/l_{ij}^2    = 6.\textrm{m_a}_k\f$
	SurgSim::Math::Vector3d m_qk;      ///< Batoz variable \f$q_{k} = 3x_{ij}y_{ij}/l_{ij}^2  = 4.\textrm{m_b}_k\f$
	SurgSim::Math::Vector3d m_tk;      ///< Batoz variable \f$t_{k} = -6y_{ij}/l_{ij}^2    = 6.\textrm{m_d}_k\f$
	SurgSim::Math::Vector3d m_rk;      ///< Batoz variable \f$r_{k} = 3y_{ij}^2/l_{ij}^2\f$
	//@}

	//@{
	SurgSim::Math::Matrix m_integral_dT_d;  ///< Plate mass matrix: integral terms related to the dof \f$(z)\f$
	SurgSim::Math::Matrix m_integralHyiHyj; ///< Plate mass matrix: integral terms related to the dof \f$(\theta_x)\f$
	SurgSim::Math::Matrix m_integralHxiHxj; ///< Plate mass matrix: integral terms related to the dof \f$(\theta_y)\f$
	//@}

	/// Batoz derivative dHx/dxi
	/// \param xi, eta The parametric coordinate (in [0 1] and xi+eta<1.0)
	/// \return The vector dHx/dxi evaluated at (xi, eta)
	std::array<double, 9> batozDhxDxi(double xi, double eta) const;
	/// Batoz derivative dHx/deta
	/// \param xi, eta The parametric coordinate (in [0 1] and xi+eta<1.0)
	/// \return The vector dHx/deta evaluated at (xi, eta)
	std::array<double, 9> batozDhxDeta(double xi, double eta) const;
	/// Batoz derivative dHy/dxi
	/// \param xi, eta The parametric coordinate (in [0 1] and xi+eta<1.0)
	/// \return The vector dHy/dxi evaluated at (xi, eta)
	std::array<double, 9> batozDhyDxi(double xi, double eta) const;
	/// Batoz derivative dHy/deta
	/// \param xi, eta The parametric coordinate (in [0 1] and xi+eta<1.0)
	/// \return The vector dHy/deta evaluated at (xi, eta)
	std::array<double, 9> batozDhyDeta(double xi, double eta) const;
	/// Batoz strain displacement matrix evaluated at a given point
	/// \param xi, eta The parametric coordinate (in [0 1] and xi+eta<1.0)
	/// \return The 3x9 strain displacement matrix evaluated at (xi, eta)
	Matrix39Type batozStrainDisplacement(double xi, double eta) const;

private:
	/// Computes the triangle's local membrane part mass matrix
	/// \param state The state to compute the local membrane part mass matrix from
	/// \param[out] localMassMatrix The local mass matrix to store the result into
	void computeLocalMembraneMass(const SurgSim::Math::OdeState& state,
								  Eigen::Matrix<double, 18, 18>* localMassMatrix);

	/// Computes the triangle's local plate part mass matrix
	/// \param state The state to compute the local plate part mass matrix from
	/// \param[out] localMassMatrix The local mass matrix to store the result into
	/// \note The plate mass matrix is composed of 3 matrices associated respectively to
	/// \note displacements in direction (z, thetax, thetay).
	void computeLocalPlateMass(const SurgSim::Math::OdeState& state,
							   Eigen::Matrix<double, 18, 18>* localMassMatrix);

	/// Computes the integral terms d^T.d over the parametrized triangle area.
	/// This integral is required in the plate mass matrix computation.
	/// The displacement along z is w(x, y) = [d1 d2 d3 d4 d5 d6 d7 d8 d9].U = d.U
	/// with di cubic shape functions and U nodal plate displacements.
	void computeIntegral_dTd();

	/// Computes the integral terms Hy.Hy^T over the parametrized triangle area.
	/// This integral is required in the plate mass matrix computation.
	/// The displacement along thetay is Thetay(x, y) = -dw/dx = betax = Hx^T.U
	/// with Hxi quadratic shape functions and U nodal plate displacements.
	void computeIntegral_HxHxT();

	/// Computes the integral terms Hx.Hx^T over the parametrized triangle area.
	/// This integral is required in the plate mass matrix computation.
	/// The displacement along thetax is Thetax(x, y) = dw/dy = -betay = -Hy^T.U
	/// with Hyi quadratic shape functions and U nodal plate displacements.
	void computeIntegral_HyHyT();
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM2DELEMENTTRIANGLE_H
