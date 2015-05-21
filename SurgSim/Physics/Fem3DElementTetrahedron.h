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

#ifndef SURGSIM_PHYSICS_FEM3DELEMENTTETRAHEDRON_H
#define SURGSIM_PHYSICS_FEM3DELEMENTTETRAHEDRON_H

#include <array>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemElementMesh.h"

namespace SurgSim
{

namespace Physics
{
SURGSIM_STATIC_REGISTRATION(Fem3DElementTetrahedron);

/// Class for Fem Element 3D based on a tetrahedron volume discretization
/// \note The inertia property (mass) of the tetrahedron is derived from
/// \note    "Theory of Matrix Structural Analysis" from J.S. Przemieniecki
/// \note The force and stiffness matrix of the tetrahedron is derived from
/// \note    http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
/// \note The deformation is based on the linear elasticity theory and not on the visco-elasticity theory.
/// \note Therefore the element does not have any damping component.
class Fem3DElementTetrahedron : public FemElement
{
public:
	/// Constructor
	/// \param nodeIds An array of 4 node ids defining this tetrahedron element in a overall mesh
	/// \note It is required that the triangle ABC is CCW looking from D (i.e. dot(cross(AB, AC), AD) > 0)
	/// \note This is required from the signed volume calculation method getVolume()
	/// \note A warning will be logged when the initialize function is called if this condition is not met, but the
	/// \note simulation will keep running.  Behavior will be undefined because of possible negative volume terms.
	explicit Fem3DElementTetrahedron(std::array<size_t, 4> nodeIds);

	/// Constructor for FemElement object factory
	/// \param elementData A FemElement3D struct defining this tetrahedron element in a overall mesh
	/// \note It is required that the triangle ABC is CCW looking from D (i.e. dot(cross(AB, AC), AD) > 0)
	/// \note This is required from the signed volume calculation method getVolume()
	/// \note A warning will be logged when the initialize function is called if this condition is not met, but the
	/// \note simulation will keep running.  Behavior will be undefined because of possible negative volume terms.
	/// \exception SurgSim::Framework::AssertionFailure if nodeIds has a size different than 4
	explicit Fem3DElementTetrahedron(std::shared_ptr<FemElementStructs::FemElement> elementData);

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem3DElementTetrahedron)

	/// Initialize the FemElement once everything has been set
	/// \param state The state to initialize the FemElement with
	/// \note We use the theory of linear elasticity, so this method pre-compute the stiffness and mass matrices
	/// \note It is required that the triangle ABC is CCW looking from D (i.e. dot(cross(AB, AC), AD) > 0)
	/// \note This is required from the signed volume calculation method getVolume()
	/// \note A warning will be logged in if this condition is not met, but the simulation will keep running.  Behavior
	/// will be undefined because of possible negative volume terms.
	void initialize(const SurgSim::Math::OdeState& state) override;

	/// Get the element volume based on the input state
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

	/// Adds the element damping matrix D (= -df/dv) (comuted for a given state)
	/// to a complete system damping matrix D (assembly)
	/// \param state The state to compute the damping matrix with
	/// \param[in,out] D The complete system damping matrix to add the element damping matrix into
	/// \param scale A factor to scale the added damping matrix with
	/// \note The element damping matrix is square of size getNumDofPerNode() x getNumNodes()
	/// \note This method supposes that the incoming state contains information with the same number of
	/// \note dof per node as getNumDofPerNode()
	/// \note Fem3DElementTetrahedron uses linear elasticity (not visco-elasticity), so it does not give any damping.
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
	void addFMDK(const SurgSim::Math::OdeState& state,
				 SurgSim::Math::Vector* F,
				 SurgSim::Math::SparseMatrix* M,
				 SurgSim::Math::SparseMatrix* D,
				 SurgSim::Math::SparseMatrix* K) override;

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

	/// Computes the tetrahedron shape functions
	/// \param state The deformable rest state to compute the shape function from
	/// \param[out] volume the volume calculated with the given state
	/// \param[out] ai from the shape function, Ni(x, y, z) = 1/6*volume (ai + bi.x + ci.y + di.z)
	/// \param[out] bi from the shape function, Ni(x, y, z) = 1/6*volume (ai + bi.x + ci.y + di.z)
	/// \param[out] ci from the shape function, Ni(x, y, z) = 1/6*volume (ai + bi.x + ci.y + di.z)
	/// \param[out] di from the shape function, Ni(x, y, z) = 1/6*volume (ai + bi.x + ci.y + di.z)
	void computeShapeFunctions(const SurgSim::Math::OdeState& state,
							   double* volume,
							   std::array<double, 4>* ai,
							   std::array<double, 4>* bi,
							   std::array<double, 4>* ci,
							   std::array<double, 4>* di) const;

	/// Computes the tetrahedron stiffness matrix
	/// \param state The state to compute the stiffness matrix from
	/// \param[out] k The stiffness matrix to store the result into
	void computeStiffness(const SurgSim::Math::OdeState& state,
						  Eigen::Matrix<double, 12, 12>* k);

	/// Computes the tetrahedron mass matrix
	/// \param state The state to compute the mass matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const SurgSim::Math::OdeState& state,
					 Eigen::Matrix<double, 12, 12>* m);

	/// Adds the element force (computed for a given state) to a complete system force vector F (assembly)
	/// This method relies on a given stiffness matrix and does not evaluate it from the state
	/// \param state The state to compute the force with
	/// \param k The given element stiffness matrix
	/// \param[in,out] F The complete system force vector to add the element force into
	/// \param scale A factor to scale the added force with
	/// \note The element force is of size (getNumDofPerNode() x getNumNodes())
	/// \note This method supposes that the incoming state contains information with the same number of dof
	/// \note per node as getNumDofPerNode()
	void addForce(const SurgSim::Math::OdeState& state, const Eigen::Matrix<double, 12, 12>& k,
				  SurgSim::Math::Vector* F, double scale = 1.0);

	/// Shape functions: Tetrahedron rest volume
	double m_restVolume;
	/// Shape functions coefficients Ni(x,y,z) = 1/6V ( ai + x.bi + y.ci + z.di )
	std::array<double, 4> m_ai, m_bi, m_ci, m_di;

	/// The tetrahedon rest state
	Eigen::Matrix<double, 12, 1> m_x0;

	/// Elasticity material matrix (contains the elastic properties of the material)
	Eigen::Matrix<double, 6, 6> m_Em;
	/// Strain matrix
	Eigen::Matrix<double, 6, 12> m_strain;
	/// Stress matrix
	Eigen::Matrix<double, 6, 12> m_stress;

	/// Mass matrix
	Eigen::Matrix<double, 12, 12> m_M;
	/// Stiffness matrix
	Eigen::Matrix<double, 12, 12> m_K;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DELEMENTTETRAHEDRON_H
