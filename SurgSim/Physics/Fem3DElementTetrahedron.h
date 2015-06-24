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
#include "SurgSim/Physics/Fem.h"
#include "SurgSim/Physics/FemElement.h"

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
	Fem3DElementTetrahedron();

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
	explicit Fem3DElementTetrahedron(std::shared_ptr<FemElementStructs::FemElementParameter> elementData);

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem3DElementTetrahedron)

	void initialize(const SurgSim::Math::OdeState& state) override;

	double getVolume(const SurgSim::Math::OdeState& state) const override;

	void addMatVec(double alphaM, double alphaD, double alphaK,
				   const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) const override;

	SurgSim::Math::Vector computeCartesianCoordinate(const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& naturalCoordinate) const override;

	SurgSim::Math::Vector computeNaturalCoordinate(const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& cartesianCoordinate) const override;

protected:
	/// Initializes variables needed before Initialize() is called
	void initializeMembers();

	void doUpdateFMDK(const Math::OdeState& state, int options) override;

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
						  SurgSim::Math::Matrix* k);

	/// Computes the tetrahedron mass matrix
	/// \param state The state to compute the mass matrix from
	/// \param[out] m The mass matrix to store the result into
	void computeMass(const SurgSim::Math::OdeState& state,
					 SurgSim::Math::Matrix* m);

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
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DELEMENTTETRAHEDRON_H
