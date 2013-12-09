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

#ifndef SURGSIM_PHYSICS_FEMELEMENT3DCUBE_H
#define SURGSIM_PHYSICS_FEMELEMENT3DCUBE_H

#include <array>

#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

/// Class for Fem Element 3D based on a cube volume discretization
/// \note The stiffness property of the cube is derived from
/// \note http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch11.d/AFEM.Ch11.pdf
/// \note The mass property of the cube is derived from the kinetic energy computed on the cube's volume
/// \note (c.f. internal documentation on cube mass matrix computation for details).
class FemElement3DCube : public FemElement
{
public:
	/// Constructor
	/// \param nodeIds An array of 8 nodes ids defining this cube element in a overall mesh
	/// \param restState The rest state to initialize the cube with
	/// \note The 8 nodes must be valid node ids in the rest state, if they aren't an ASSERT will be raised
	/// \note The 8 nodes must defined a cube with positive volume, if they don't an ASSERT will be raised
	FemElement3DCube(std::array<unsigned int, 8> nodeIds, const DeformableRepresentationState& restState);

	/// Initializes the element once everything has been set
	/// \param state The state to initialize the FemElement with
	/// \note We use the theory of linear elasticity, so this method pre-compute the stiffness and mass matrices
	virtual void initialize(const DeformableRepresentationState& state) override;

	/// Gets the element volume based on the input state
	/// \param state The deformable state to compute the volume with
	virtual double getVolume(const DeformableRepresentationState& state) const override;

	/// Adds the element force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param scale A factor to scale the added force with
	/// \note The element force is of size (getNumDofPerNode() x getNumNodes())
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	virtual void addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F,
		double scale = 1.0) override;

	/// Adds the element mass matrix M (computed for a given state) to a complete system mass matrix M (assembly)
	/// \param state The state to compute the mass matrix with
	/// \param[in,out] M The complete system mass matrix to add the element mass-matrix into
	/// \param scale A factor to scale the added mass matrix with
	/// \note The element mass matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual void addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M,
		double scale = 1.0) override;

	/// Adds the element damping matrix D (= -df/dv) (comuted for a given state)
	/// to a complete system damping matrix D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	/// \note The element damping matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	/// \note FemElement3DCube uses linear elasticity (not visco-elasticity), so it does not give any damping.
	virtual void addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D,
		double scale = 1.0) override;

	/// Adds the element stiffness matrix K (= -df/dx) (computed for a given state)
	/// to a complete system stiffness matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	virtual void addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K,
		double scale = 1.0) override;

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

	/// Adds the element matrix-vector contribution F += (alphaM.M + alphaD.D + alphaK.K).x (computed for a given state)
	/// into a complete system data structure F (assembly)
	/// \param state The state to compute everything with
	/// \param alphaM The scaling factor for the mass contribution
	/// \param alphaD The scaling factor for the damping contribution
	/// \param alphaK The scaling factor for the stiffness contribution
	/// \param x A complete system vector to use as the vector in the matrix-vector multiplication
	/// \param[in,out] F The complete system force vector to add the element matrix-vector contribution into
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	virtual void addMatVec(const DeformableRepresentationState& state,
		double alphaM, double alphaD, double alphaK,
		const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) override;

protected:
	/// Computes the cube stiffness matrix
	/// \param state The deformable state to compute the stiffness matrix from
	/// \param[out] strain The strain-displacement matrix
	/// \param[out] stress The stress matrix
	/// \param[out] k The stiffness matrix to store the result into
	void computeStiffness(const DeformableRepresentationState& state,
		Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* strain,
		Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* stress,
		Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* k);

	/// Computes the cube mass matrix
	/// \param state The deformable state to compute the mass matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const DeformableRepresentationState& state,
		Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* m);

	/// Adds the element force (computed for a given state) to a complete system force vector F (assembly)
	/// This method relies on a given stiffness matrix and does not evaluate it from the state
	/// \param state The state to compute the force with
	/// \param k The given element stiffness matrix
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param scale A factor to scale the added force with
	/// \note The element force is of size (getNumDofPerNode() x getNumNodes())
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	void addForce(const DeformableRepresentationState& state, const Eigen::Matrix<double, 24, 24>& k,
		SurgSim::Math::Vector* F, double scale = 1.0);

	/// Helper method to evaluate strain-stress and stiffness integral terms with a discrete sum using
	/// a Gauss quadrature rule
	/// \param state The state to compute the evaluation with
	/// \param epsilon, neta, mu The 3D parametric coordinates to evaluate the data at (within [-1 +1])
	/// \param weightEpsilon, weightNeta, weightMu The weight to apply to this evaluation point in the Gauss quadrature
	/// \param[out] strain, stress, k The matrices in which to add the evaluations
	void addStrainStressStiffnessAtPoint(const DeformableRepresentationState& state,
		double epsilon, double neta, double mu,
		double weightEpsilon, double weightNeta, double weightMu,
		Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* strain,
		Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* stress,
		Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* k);

	/// Helper method to evaluate mass integral terms with a discrete sum using a Gauss quadrature rule
	/// \param state The state to compute the evaluation with
	/// \param epsilon, neta, mu The 3D parametric coordinates to evaluate the data at (within [-1 +1])
	/// \param weightEpsilon, weightNeta, weightMu The weight to apply to this evaluation point in the Gauss quadrature
	/// \param[out] m The matrix in which to add the evaluations
	void addMassMatrixAtPoint(const DeformableRepresentationState& state,
		double epsilon, double neta, double mu,
		double weightEpsilon, double weightNeta, double weightMu,
		Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* m);

	/// Helper method to evaluate the J matrix at a given 3D parametric location
	/// J is a matrix that expresses the element 3D global coords into 3D parametric coords
	/// \param state The state to compute the evaluation with
	/// \param epsilon, neta, mu The 3D parametric coordinates to evaluate the data at (within [-1 +1])
	/// \param[out] J, Jinv, detJ The J matrix with its inverse and determinant evaluated at (epsilon, neta, mu)
	void evaluateJ(const DeformableRepresentationState& state, double epsilon, double neta, double mu,
		SurgSim::Math::Matrix33d *J,
		SurgSim::Math::Matrix33d *Jinv,
		double *detJ) const;

	/// Helper method to evaluate the B (strain-displacement) matrix at a given 3D parametric location
	/// \param epsilon, neta, mu The 3D parametric coordinates to evaluate the data at (within [-1 +1])
	/// \param Jinv The inverse of matrix J (3D global coords to 3D parametric coords)
	/// \param[out] B The the strain-displacement matrix
	void evaluateB(double epsilon, double neta, double mu, const SurgSim::Math::Matrix33d& Jinv,
		Eigen::Matrix<double, 6, 24, Eigen::DontAlign> *B) const;

	/// Cube rest volume
	double m_restVolume;

	/// Shape functions parameters
	/// Ni(epsilon, neta, mu) = (1+-epsilon)(1+-neta)(1+-mu)/8
	///                       = (1+epsilon.m_coefEpsilon[i])(1+neta.m_coefNeta[i])(1+mu.m_coefMu[i])/8
	/// The 3D parametric coordinates (epsilon, neta, mu) are each within [-1 +1]
	std::array<double, 8> m_coefEpsilon;
	std::array<double, 8> m_coefNeta;
	std::array<double, 8> m_coefMu;

	/// Shape functions Ni(epsilon, neta, mu) = (1+-epsilon)(1+-neta)(1+-mu)/8
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, neta, mu The 3D parametric coordinates each within [-1 +1]
	/// \return Ni(epsilon, neta, mu)
	/// \note No check is done on the nodeId i nor the 3D parametric coordinates
	double N(size_t i, double epsilon, double neta, double mu) const;

	/// Shape functions derivative dNi/depsilon(epsilon, neta, mu) = +-(1+-neta)(1+-mu)/8
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, neta, mu The 3D parametric coordinates each within [-1 +1]
	/// \return dNi/depsilon(epsilon, neta, mu)
	/// \note No check is done on the nodeId i nor the 3D parametric coordinates
	double dNdepsilon(size_t i, double epsilon, double neta, double mu) const;

	/// Shape functions derivative dNi/dneta(epsilon, neta, mu) = +-(1+-epsilon)(1+-mu)/8
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, neta, mu The 3D parametric coordinates each within [-1 +1]
	/// \return dNi/depsilon(epsilon, neta, mu)
	/// \note No check is done on the nodeId i nor the 3D parametric coordinates
	double dNdneta(size_t i, double epsilon, double neta, double mu) const;

	/// Shape functions derivative dNi/dmu(epsilon, neta, mu) = +-(1+-epsilon)(1+-neta)/8
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, neta, mu The 3D parametric coordinates each within [-1 +1]
	/// \return dNi/depsilon(epsilon, neta, mu)
	/// \note No check is done on the nodeId i nor the 3D parametric coordinates
	double dNdmu(size_t i, double epsilon, double neta, double mu) const;

	/// The cube rest state (nodes ordered by m_nodeIds)
	Eigen::Matrix<double, 24, 1, Eigen::DontAlign> m_x0;

	/// Elasticity material matrix (contains the elastic properties of the material)
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> m_Em;
	/// Strain matrix
	Eigen::Matrix<double, 6, 24, Eigen::DontAlign> m_strain;
	/// Stress matrix
	Eigen::Matrix<double, 6, 24, Eigen::DontAlign> m_stress;

	/// Mass matrix
	Eigen::Matrix<double, 24, 24, Eigen::DontAlign> m_M;
	/// Stiffness matrix
	Eigen::Matrix<double, 24, 24, Eigen::DontAlign> m_K;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENT3DCUBE_H
