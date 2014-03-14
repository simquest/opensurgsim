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

#ifndef SURGSIM_PHYSICS_FEMELEMENT2DTRIANGLE_H
#define SURGSIM_PHYSICS_FEMELEMENT2DTRIANGLE_H

#include <array>

#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

/// 2D FemElement based on a triangle with a constant thickness
///
/// XXX TO DO
/// XXX Reference Batoz for deformation
/// XXX Reference Prezienski for mass
/// \note The element is considered to have a constant thickness.
class FemElement2DTriangle : public FemElement
{
public:
	/// Constructor
	/// \param nodeIds An array of 3 node ids (A, B, C) defining this triangle element with respect to a
	/// DeformableRepresentaitonState which is passed to the initialize method.
	FemElement2DTriangle(std::array<unsigned int, 3> nodeIds);

	/// Sets the triangle's thickness
	/// \param thickness The thickness of the triangle
	void setThickness(double thickness);

	/// Gets the triangle's thickness
	/// \return The thickness of the triangle
	double getThickness() const;

	/// Initializes the FemElement once everything has been set
	/// \param state The state to initialize the FemElement with
	/// \note We use the theory of linear elasticity, so this method pre-computes the stiffness and mass matrices
	virtual void initialize(const DeformableRepresentationState& state) override;

	/// Gets the element's volume based on the input state
	/// \param state The deformable state to compute the volume with
	/// \return The element's volume
	virtual double getVolume(const DeformableRepresentationState& state) const override;

	/// Adds the element's force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the element's force into
	/// \param scale A factor to scale the added force with
	/// \note The element's force is of size (getNumDofPerNode() x getNumNodes()).
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode().
	virtual void addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F,
						  double scale = 1.0) override;

	/// Adds the element's mass matrix M (computed for a given state) to a complete system mass matrix M (assembly)
	/// \param state The state to compute the mass matrix with
	/// \param[in,out] M The complete system mass matrix to add the element's mass-matrix into
	/// \param scale A factor to scale the added mass matrix with
	/// \note The element's mass matrix is a square matrix of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	virtual void addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M,
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
	virtual void addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D,
							double scale = 1.0) override;

	/// Adds the element's stiffness matrix K (= -df/dx) (computed for a given state) to a complete system stiffness
	/// matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes().
	/// \note This method supposes that the incoming state contains information with the same number of dof per node as
	/// getNumDofPerNode()
	virtual void addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K,
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
	virtual void addFMDK(const DeformableRepresentationState& state, SurgSim::Math::Vector* F, SurgSim::Math::Matrix* M,
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
	virtual void addMatVec(const DeformableRepresentationState& state, double alphaM, double alphaD, double alphaK,
						   const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F);

	/// Determines whether a given natural coordinate is valid
	/// \param naturalCoordinate Coordinate to check
	/// \return True if valid
	virtual bool isValidCoordinate(const SurgSim::Math::Vector& naturalCoordinate) const;

	/// Computes a given natural coordinate in cartesian coordinates
	/// \param state The state at which to transform coordinates
	/// \param naturalCoordinate The coordinates to transform
	/// \return The resultant cartesian coordinates
	virtual SurgSim::Math::Vector computeCartesianCoordinate(const DeformableRepresentationState& state,
															 const SurgSim::Math::Vector& naturalCoordinate) const;

protected:
	/// Computes the triangle element's initial rotation
	/// \param state The deformable state to compute the rotation from
	/// \note This method stores the result in m_R0
	void computeInitialRotation(const DeformableRepresentationState& state);

	/// Computes the triangle's stiffness matrix
	/// \param state The deformable state to compute the stiffness matrix from
	/// \param[out] k The stiffness matrix to store the result into
	void computeStiffness(const DeformableRepresentationState& state,
		Eigen::Matrix<double, 18, 18, Eigen::DontAlign>* k);

	/// Computes the triangle's mass matrix
	/// \param state The deformable state to compute the stiffness matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const DeformableRepresentationState& state, Eigen::Matrix<double, 18, 18, Eigen::DontAlign>* m);

	/// The element's rest state
	Eigen::Matrix<double, 18, 1, Eigen::DontAlign> m_x0;

	/// Initial rotation matrix for the element
	Eigen::Matrix<double, 18, 18, Eigen::DontAlign> m_R0;

	/// Mass matrix (in global coordinate frame)
	Eigen::Matrix<double, 18, 18, Eigen::DontAlign> m_M;
	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 18, 18, Eigen::DontAlign> m_MLocal;
	/// Stiffness matrix (in global coordinate frame)
	Eigen::Matrix<double, 18, 18, Eigen::DontAlign> m_K;
	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 18, 18, Eigen::DontAlign> m_KLocal;

	/// Physical shear modulus G = E/( 2(1+mu) )
	double m_G;

	// The triangle rest area
	double m_restArea;

	/// thickness of the element
	double m_thickness;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENT2DTRIANGLE_H
