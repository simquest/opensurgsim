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

#ifndef SURGSIM_PHYSICS_FEM1DELEMENTBEAM_H
#define SURGSIM_PHYSICS_FEM1DELEMENTBEAM_H

#include <array>

#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

SURGSIM_STATIC_REGISTRATION(Fem1DElementBeam);

/// 1D FemElement based on a beam volume discretization with a fixed cross section
///
/// The inertia property (mass) and the stiffness matrices are derived from "Theory of Matrix Structural Analysis" from
/// J.S. Przemieniecki.  The deformation is based on linear elasticity theory and not on visco-elasticity theory;
/// therefore, the element does not have any damping components.
/// \note The element is considered to have a circular cross section.
class Fem1DElementBeam : public FemElement
{
public:
	/// Constructor
	/// \param nodeIds An array of 2 node ids defining this beam element with respect to a
	/// DeformableRepresentaitonState which is passed to the initialize method.
	explicit Fem1DElementBeam(std::array<size_t, 2> nodeIds);

	/// Constructor for FemElement object factory
	/// \param nodeIds A vector of node ids defining this beam element with respect to a
	/// DeformableRepresentaitonState which is passed to the initialize method.
	/// \exception SurgSim::Framework::AssertionFailure if nodeIds has a size different than 2
	explicit Fem1DElementBeam(std::shared_ptr<FemElementStructs::FemElement> elementData);

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem1DElementBeam)

	/// Sets the beam's circular cross-section radius
	/// \param radius The radius of the beam
	void setRadius(double radius);

	/// Gets the beam's circular cross-section radius
	/// \return The radius of the beam
	double getRadius() const;

	/// Initializes the FemElement once everything has been set
	/// \param state The state to initialize the FemElement with
	/// \note We use the theory of linear elasticity, so this method pre-computes the stiffness and mass matrices
	void initialize(const SurgSim::Math::OdeState& state) override;

	/// Gets the element's volume based on the input state
	/// \param state The state to compute the volume with
	/// \return The element's volume
	double getVolume(const SurgSim::Math::OdeState& state) const override;

	/// Gets whether shearing is enabled for the element
	/// \return True if shearing is enabled
	bool getShearingEnabled() const;

	/// Enables or disables shearing for the element
	///
	/// Shearing can only be meaningfully enabled or disabled before the element has had initialize called.
	/// \param enabled Boolean determining whether shearing is enabled
	void setShearingEnabled(bool enabled);

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
	void addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* M, double scale = 1.0) override;

	/// Adds the element's damping matrix D (= -df/dv) (computed for a given state) to a complete system damping matrix
	/// D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	/// \note The element's damping matrix is a square matrix of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	/// \note The beam uses linear elasticity (not visco-elasticity), so it does not have any damping.
	void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* D, double scale = 1.0) override;

	/// Adds the element's stiffness matrix K (= -df/dx) (computed for a given state) to a complete system stiffness
	/// matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* K,
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
	void addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, SurgSim::Math::SparseMatrix* M,
				 SurgSim::Math::SparseMatrix* D, SurgSim::Math::SparseMatrix* K) override;

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

	SurgSim::Math::Vector computeNaturalCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& cartesianCoordinate) const override;

protected:
	/// Initializes variables needed before Initialize() is called
	void init();

	/// Computes the beam element's initial rotation
	/// \param state The state to compute the rotation from
	/// \note This method stores the result in m_R0
	void computeInitialRotation(const SurgSim::Math::OdeState& state);

	/// Computes the beam's stiffness matrix
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] k The stiffness matrix to store the result into
	void computeStiffness(const SurgSim::Math::OdeState& state,
						  Eigen::Matrix<double, 12, 12>* k);

	/// Computes the beam's mass matrix
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const SurgSim::Math::OdeState& state, Eigen::Matrix<double, 12, 12>* m);

	/// The element's rest state
	Eigen::Matrix<double, 12, 1> m_x0;

	/// Initial rotation matrix for the element
	Eigen::Matrix<double, 12, 12> m_R0;

	/// Mass matrix (in global coordinate frame)
	Eigen::Matrix<double, 12, 12> m_M;
	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 12, 12> m_MLocal;
	/// Stiffness matrix (in global coordinate frame)
	Eigen::Matrix<double, 12, 12> m_K;
	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 12, 12> m_KLocal;

	/// Physical shear modulus G = E/( 2(1+mu) )
	double m_G;

	/// Rest length
	double m_restLength;
	/// radius for a circular Beam
	double m_radius;
	/// Cross sectional area = PI.radius.radius if circular
	double m_A;
	/// Does this beam element have shear
	bool m_haveShear;
	/// Shear factor (usually 5/8)
	double m_shearFactor;
	/// The shear area in the y and z directions (=0 => no shear) http://en.wikipedia.org/wiki/Timoshenko_beam_theory
	double m_Asy, m_Asz;
	/// Shear deformation parameters
	/// Phi_y=12.E.Iz/(G.Asy.L^2) or 0 if As?=0
	/// Phi_z=12.E.Iy/(G.Asz.L^2) or 0 if As?=0
	double m_Phi_y, m_Phi_z;
	/// Cross sectional moment of inertia
	double m_Iy, m_Iz;
	/// Polar moment of inertia
	double m_J;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM1DELEMENTBEAM_H
