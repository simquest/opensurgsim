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
	Fem1DElementBeam();

	/// Constructor
	/// \param nodeIds An array of 2 node ids defining this beam element with respect to a
	/// DeformableRepresentationState which is passed to the initialize method.
	explicit Fem1DElementBeam(std::array<size_t, 2> nodeIds);

	/// Constructor for FemElement object factory
	/// \param elementData A FemElement struct defining this beam element with respect to a
	/// DeformableRepresentationState which is passed to the initialize method.
	/// \exception SurgSim::Framework::AssertionFailure if nodeIds has a size different than 2
	explicit Fem1DElementBeam(std::shared_ptr<FemElementStructs::FemElementParameter> elementData);

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem1DElementBeam)

	/// Sets the beam's circular cross-section radius
	/// \param radius The radius of the beam
	void setRadius(double radius);

	/// Gets the beam's circular cross-section radius
	/// \return The radius of the beam
	double getRadius() const;

	void initialize(const SurgSim::Math::OdeState& state) override;

	double getVolume(const SurgSim::Math::OdeState& state) const override;

	/// Gets whether shearing is enabled for the element
	/// \return True if shearing is enabled
	bool getShearingEnabled() const;

	/// Enables or disables shearing for the element
	///
	/// Shearing can only be meaningfully enabled or disabled before the element has had initialize called.
	/// \param enabled Boolean determining whether shearing is enabled
	void setShearingEnabled(bool enabled);

	void addMatVec(double alphaM, double alphaD, double alphaK, const SurgSim::Math::Vector& x,
				   SurgSim::Math::Vector* F) const override;

	SurgSim::Math::Vector computeCartesianCoordinate(
			const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& naturalCoordinate) const override;

	SurgSim::Math::Vector computeNaturalCoordinate(
			const SurgSim::Math::OdeState& state,
			const SurgSim::Math::Vector& cartesianCoordinate) const override;

protected:
	/// Initializes variables needed before Initialize() is called
	void initializeMembers();

	/// Computes the beam element's initial rotation
	/// \param state The state to compute the rotation from
	/// \note This method stores the result in m_R0
	void computeInitialRotation(const SurgSim::Math::OdeState& state);

	/// Computes the beam's stiffness matrix
	/// \param state The state to compute the stiffness matrix from
	void computeStiffness(const SurgSim::Math::OdeState& state);

	/// Computes the beam's mass matrix
	/// \param state The state to compute the stiffness matrix from
	void computeMass(const SurgSim::Math::OdeState& state);

	void doUpdateFMDK(const Math::OdeState& state, int options) override;

	/// The element's rest state
	Eigen::Matrix<double, 12, 1> m_x0;

	/// Initial rotation matrix for the element
	Eigen::Matrix<double, 12, 12> m_R0;

	/// Stiffness matrix (in local coordinate frame)
	Eigen::Matrix<double, 12, 12> m_MLocal;
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
