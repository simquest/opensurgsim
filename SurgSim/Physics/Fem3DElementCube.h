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

#ifndef SURGSIM_PHYSICS_FEM3DELEMENTCUBE_H
#define SURGSIM_PHYSICS_FEM3DELEMENTCUBE_H

#include <array>

#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Math/GaussLegendreQuadrature.h"

namespace SurgSim
{

namespace Physics
{
SURGSIM_STATIC_REGISTRATION(Fem3DElementCube);

/// Class for Fem Element 3D based on a cube volume discretization
/// \note The stiffness property of the cube is derived from
/// \note http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch11.d/AFEM.Ch11.pdf
/// \note The mass property of the cube is derived from the kinetic energy computed on the cube's volume
/// \note (c.f. internal documentation on cube mass matrix computation for details).
/// \note Integral terms over the volume are evaluated using the Gauss-Legendre 2-points quadrature.
/// \note http://en.wikipedia.org/wiki/Gaussian_quadrature
/// \note Note that this technique is accurate for any polynomial evaluation up to degree 3.
/// \note In our case, the shape functions \f$N_i\f$ are linear (of degree 1). So for exmaple,
/// \note in the mass matrix we have integral terms like \f$\int_V N_i.N_j dV\f$, which are of degree 2.
class Fem3DElementCube : public FemElement
{
public:
	/// Constructor
	Fem3DElementCube();

	/// Constructor
	/// \param nodeIds An array of 8 node ids defining this cube element in an overall mesh
	/// \note It is required that getVolume() is positive, to do so, it needs (looking at the cube from
	/// \note the exterior, face normal 'n' pointing outward):
	/// \note   the 1st  4 nodeIds (ABCD) should define any face CW            i.e. (AB^AC or AB^AD or AC^AD).n < 0
	/// \note   the last 4 nodeIds (EFGH) should define the opposite face CCW  i.e. (EF^EG or EF^EH or EG^EH).n > 0
	/// \note A warning will be logged when the initialize function is called if this condition is not met, but the
	/// \note simulation will keep running.  Behavior will be undefined because of possible negative volume terms.
	explicit Fem3DElementCube(std::array<size_t, 8> nodeIds);

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem3DElementCube)

	/// Set the element data
	/// \param data A MeshElement that conatins all required Fem3D data
	void setData(std::vector<size_t> nodeIds, std::shared_ptr<FemElementStructs::FemElementParameter> data) override;

	/// Initializes the element once everything has been set
	/// \param state The state to initialize the FemElement with
	/// \note We use the theory of linear elasticity, so this method precomputes the stiffness and mass matrices
	/// \note The 8 node ids must be valid in the given state, if they aren't an ASSERT will be raised
	/// \note The 8 node ids must define a cube with positive volume, if they don't an ASSERT will be raised
	/// \note In order to do so (looking at the cube from the exterior, face normal 'n' pointing outward),
	/// \note   the 1st  4 nodeIds (ABCD) should define any face CW            i.e. (AB^AC or AB^AD or AC^AD).n < 0
	/// \note   the last 4 nodeIds (EFGH) should define the opposite face CCW  i.e. (EF^EG or EF^EH or EG^EH).n > 0
	/// \note A warning will be logged in if this condition is not met, but the simulation will keep running.
	/// \note Behavior will be undefined because of possible negative volume terms.
	void initialize(const SurgSim::Math::OdeState& state) override;

	/// Gets the element volume based on the input state
	/// \param state The state to compute the volume with
	double getVolume(const SurgSim::Math::OdeState& state) const override;

	/// Adds the element force (computed for a given state) to a complete system force vector F (assembly)
	/// \param state The state to compute the force with
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param scale A factor to scale the added force with
	/// \note The element force is of size (getNumDofPerNode() x getNumNodes())
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale = 1.0) override;

	/// Adds the element mass matrix M (computed for a given state) to a complete system mass matrix M (assembly)
	/// \param state The state to compute the mass matrix with
	/// \param[in,out] M The complete system mass matrix to add the element mass-matrix into
	/// \param scale A factor to scale the added mass matrix with
	/// \note The element mass matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	void addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* M, double scale = 1.0) override;

	/// Adds the element damping matrix D (= -df/dv) (computed for a given state)
	/// to a complete system damping matrix D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	/// \note The element damping matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	/// \note Fem3DElementCube uses linear elasticity (not visco-elasticity), so it does not give any damping.
	void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* D, double scale = 1.0) override;

	/// Adds the element stiffness matrix K (= -df/dx) (computed for a given state)
	/// to a complete system stiffness matrix K (assembly)
	/// \param state The state to compute the stiffness matrix with
	/// \param[in,out] K The complete system stiffness matrix to add the element stiffness matrix into
	/// \param scale A factor to scale the added stiffness matrix with
	/// \note The element stiffness matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* K,
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
	void addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, SurgSim::Math::SparseMatrix* M,
				 SurgSim::Math::SparseMatrix* D, SurgSim::Math::SparseMatrix* K) override;

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
	void addMatVec(const SurgSim::Math::OdeState& state, double alphaM, double alphaD, double alphaK,
				   const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) override;

	SurgSim::Math::Vector computeCartesianCoordinate(const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& naturalCoordinate) const override;

	SurgSim::Math::Vector computeNaturalCoordinate(const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& cartesianCoordinate) const override;

protected:
	/// Initializes variables needed before Initialize() is called
	void init();

	/// Build the constitutive material 6x6 matrix
	/// \param[out] constitutiveMatrix The 6x6 constitutive material matrix
	void buildConstitutiveMaterialMatrix(Eigen::Matrix<double, 6, 6>* constitutiveMatrix);

	/// Computes the cube stiffness matrix along with the strain and stress matrices
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] strain, stress, k The strain, stress and stiffness matrices to store the result into
	void computeStiffness(const SurgSim::Math::OdeState& state,
						  Eigen::Matrix<double, 6, 24>* strain,
						  Eigen::Matrix<double, 6, 24>* stress,
						  Eigen::Matrix<double, 24, 24>* k);

	/// Computes the cube mass matrix
	/// \param state The state to compute the mass matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const SurgSim::Math::OdeState& state, Eigen::Matrix<double, 24, 24>* m);

	/// Adds the element force (computed for a given state) to a complete system force vector F (assembly)
	/// This method relies on a given stiffness matrix and does not evaluate it from the state
	/// \param state The state to compute the force with
	/// \param k The given element stiffness matrix
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param scale A factor to scale the added force with
	/// \note The element force is of size (getNumDofPerNode() x getNumNodes())
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	void addForce(const SurgSim::Math::OdeState& state, const Eigen::Matrix<double, 24, 24>& k,
				  SurgSim::Math::Vector* F, double scale = 1.0);

	/// Helper method to evaluate strain-stress and stiffness integral terms with a discrete sum using
	/// a Gauss quadrature rule
	/// \param state The state to compute the evaluation with
	/// \param epsilon, eta, mu The Gauss quadrature points to evaluate the data at
	/// \param[out] strain, stress, k The matrices in which to add the evaluations
	void addStrainStressStiffnessAtPoint(const SurgSim::Math::OdeState& state,
										 const SurgSim::Math::gaussQuadraturePoint& epsilon,
										 const SurgSim::Math::gaussQuadraturePoint& eta,
										 const SurgSim::Math::gaussQuadraturePoint& mu,
										 Eigen::Matrix<double, 6, 24>* strain,
										 Eigen::Matrix<double, 6, 24>* stress,
										 Eigen::Matrix<double, 24, 24>* k);

	/// Helper method to evaluate mass integral terms with a discrete sum using a Gauss quadrature rule
	/// \param state The state to compute the evaluation with
	/// \param epsilon, eta, mu The Gauss quadrature points to evaluate the data at
	/// \param[out] m The matrix in which to add the evaluations
	void addMassMatrixAtPoint(const SurgSim::Math::OdeState& state,
							  const SurgSim::Math::gaussQuadraturePoint& epsilon,
							  const SurgSim::Math::gaussQuadraturePoint& eta,
							  const SurgSim::Math::gaussQuadraturePoint& mu,
							  Eigen::Matrix<double, 24, 24>* m);

	/// Helper method to evaluate matrix J = d(x,y,z)/d(epsilon,eta,mu) at a given 3D parametric location
	/// J expresses the 3D space coordinate frames variation w.r.t. parametric coordinates
	/// \param state The state to compute the evaluation with
	/// \param epsilon, eta, mu The 3D parametric coordinates to evaluate the data at (within \f$[-1 +1]\f$)
	/// \param[out] J, Jinv, detJ The J matrix with its inverse and determinant evaluated at (epsilon, eta, mu)
	void evaluateJ(const SurgSim::Math::OdeState& state, double epsilon, double eta, double mu,
				   SurgSim::Math::Matrix33d* J,
				   SurgSim::Math::Matrix33d* Jinv,
				   double* detJ) const;

	/// Helper method to evaluate the strain-displacement matrix at a given 3D parametric location
	/// c.f. http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch11.d/AFEM.Ch11.pdf for more details
	/// \param epsilon, eta, mu The 3D parametric coordinates to evaluate the data at (within \f$[-1 +1]\f$)
	/// \param Jinv The inverse of matrix J (3D global coords to 3D parametric coords)
	/// \param[out] B The strain-displacement matrix
	void evaluateStrainDisplacement(double epsilon, double eta, double mu, const SurgSim::Math::Matrix33d& Jinv,
									Eigen::Matrix<double, 6, 24>* B) const;

	/// Cube rest volume
	double m_restVolume;

	///@{
	/// Shape functions parameters
	/// \f$N_i(\epsilon, \eta, \mu) = (1\pm\epsilon)(1\pm\eta)(1\pm\mu)/8
	///   = (1+\epsilon.sign(\epsilon_i))(1+\eta.sign(\eta_i))(1+\mu.sign(\mu_i))/8
	///   \textbf{ with } (\epsilon, \eta, \mu) \in [-1 +1]^3\f$
	///
	/// We choose to only store the sign of epsilon, eta and mu for each shape functions.
	/// \sa N
	std::array<double, 8> m_shapeFunctionsEpsilonSign;
	std::array<double, 8> m_shapeFunctionsEtaSign;
	std::array<double, 8> m_shapeFunctionsMuSign;
	///@}

	/// Shape functions \f$N_i(\epsilon, \eta, \mu) = (1\pm\epsilon)(1\pm\eta)(1\pm\mu)/8\f$
	///
	/*! \f$
	 *  \begin{array}{r | r r r}
	 *  i & sign(\epsilon) & sign(\eta) & sign(\mu) \\
	 *   \hline
	 *   0 & -1 & -1 & -1 \\
	 *      1 & +1 & -1 & -1 \\
	 *      2 & +1 & +1 & -1 \\
	 *      3 & -1 & +1 & -1 \\
	 *      4 & -1 & -1 & +1 \\
	 *      5 & +1 & -1 & +1 \\
	 *      6 & +1 & +1 & +1 \\
	 *      7 & -1 & +1 & +1
	 *  \end{array}
	 * \f$
	 */
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, eta, mu The 3D parametric coordinates each within \f$[-1 +1]\f$
	/// \return Ni(epsilon, eta, mu)
	/// \note A check is performed on the nodeId i but not on the 3D parametric coordinates range
	/// \sa m_shapeFunctionsEpsilonSign, m_shapeFunctionsEtaSign, m_shapeFunctionsMuSign
	/// \sa dNdepsilon, dNdeta, dNdmu
	double shapeFunction(size_t i, double epsilon, double eta, double mu) const;

	/// Shape functions derivative \f$dN_i/d\epsilon(\epsilon, \eta, \mu) = \pm(1\pm\eta)(1\pm\mu)/8\f$
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, eta, mu The 3D parametric coordinates each within \f$[-1 +1]\f$
	/// \return dNi/depsilon(epsilon, eta, mu)
	/// \note A check is performed on the nodeId i but not on the 3D parametric coordinates range
	/// \sa N
	/// \sa m_shapeFunctionsEpsilonSign, m_shapeFunctionsEtaSign, m_shapeFunctionsMuSign
	double dShapeFunctiondepsilon(size_t i, double epsilon, double eta, double mu) const;

	/// Shape functions derivative \f$dN_i/d\eta(\epsilon, \eta, \mu) = \pm(1\pm\epsilon)(1\pm\mu)/8\f$
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, eta, mu The 3D parametric coordinates each within \f$[-1 +1]\f$
	/// \return dNi/depsilon(epsilon, eta, mu)
	/// \note A check is performed on the nodeId i but not on the 3D parametric coordinates range
	/// \sa N
	/// \sa m_shapeFunctionsEpsilonSign, m_shapeFunctionsEtaSign, m_shapeFunctionsMuSign
	double dShapeFunctiondeta(size_t i, double epsilon, double eta, double mu) const;

	/// Shape functions derivative \f$dN_i/d\mu(\epsilon, \eta, \mu) = \pm(1\pm\epsilon)(1\pm\eta)/8\f$
	/// \param i The node id (w.r.t. local element) to evaluate at
	/// \param epsilon, eta, mu The 3D parametric coordinates each within \f$[-1 +1]\f$
	/// \return dNi/depsilon(epsilon, eta, mu)
	/// \note A check is performed on the nodeId i but not on the 3D parametric coordinates range
	/// \sa N
	/// \sa m_shapeFunctionsEpsilonSign, m_shapeFunctionsEtaSign, m_shapeFunctionsMuSign
	double dShapeFunctiondmu(size_t i, double epsilon, double eta, double mu) const;

	/// The cube rest state (nodes ordered by m_nodeIds)
	Eigen::Matrix<double, 24, 1> m_elementRestPosition;

	/// Strain matrix (usually noted \f$\epsilon\f$)
	Eigen::Matrix<double, 6, 24> m_strain;
	/// Stress matrix (usually noted \f$\sigma\f$)
	Eigen::Matrix<double, 6, 24> m_stress;
	/// Constitutive material matrix (Hooke's law in this case) defines the relationship between stress and strain
	Eigen::Matrix<double, 6, 6> m_constitutiveMaterial;

	/// %Mass matrix (usually noted \f$M\f$)
	Eigen::Matrix<double, 24, 24> m_mass;
	/// Stiffness matrix (usually noted \f$K\f$)
	Eigen::Matrix<double, 24, 24> m_stiffness;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DELEMENTCUBE_H
