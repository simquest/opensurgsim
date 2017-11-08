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

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"

using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubMatrix;
using SurgSim::Math::getSubVector;
using SurgSim::Math::setSubMatrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{
SURGSIM_REGISTER(SurgSim::Physics::FemElement, SurgSim::Physics::Fem1DElementBeam, Fem1DElementBeam)

Fem1DElementBeam::Fem1DElementBeam()
{
	initializeMembers();
}

Fem1DElementBeam::Fem1DElementBeam(std::array<size_t, 2> nodeIds)
{
	initializeMembers();
	m_nodeIds.assign(nodeIds.cbegin(), nodeIds.cend());
}

Fem1DElementBeam::Fem1DElementBeam(std::shared_ptr<FemElementStructs::FemElementParameter> elementData)
{
	initializeMembers();
	auto element1DData = std::dynamic_pointer_cast<FemElementStructs::FemElement1DParameter>(elementData);
	SURGSIM_ASSERT(element1DData != nullptr) << "Incorrect struct type passed";
	SURGSIM_ASSERT(element1DData->nodeIds.size() == 2) << "Incorrect number of nodes for a Fem1D Beam";
	m_nodeIds.assign(element1DData->nodeIds.begin(), element1DData->nodeIds.end());
	setShearingEnabled(element1DData->enableShear);
	setRadius(element1DData->radius);
	setMassDensity(element1DData->massDensity);
	setPoissonRatio(element1DData->poissonRatio);
	setYoungModulus(element1DData->youngModulus);
}

void Fem1DElementBeam::setRadius(double radius)
{
	SURGSIM_ASSERT(radius != 0.0) << "The beam radius cannot be set to 0";
	SURGSIM_ASSERT(radius > 0.0) << "The beam radius cannot be negative (trying to set it to " << radius << ")";

	m_radius = radius;
}

double Fem1DElementBeam::getRadius() const
{
	return m_radius;
}

bool Fem1DElementBeam::getShearingEnabled() const
{
	return m_haveShear;
}

void Fem1DElementBeam::setShearingEnabled(bool enabled)
{
	m_haveShear = enabled;
}

double Fem1DElementBeam::getVolume(const SurgSim::Math::OdeState& state) const
{
	const Vector3d A = state.getPosition(m_nodeIds[0]);
	const Vector3d B = state.getPosition(m_nodeIds[1]);

	return m_A * (B - A).norm();
}

void Fem1DElementBeam::initializeMembers()
{
	m_restLength = 0.0;
	m_radius = 0.0;
	m_A = 0.0;
	m_haveShear = true;
	m_shearFactor = (5.0 / 8.0);
	m_J = 0.0;

	// 6 dof per node (x, y, z, thetaX, thetaY, thetaZ)
	setNumDofPerNode(6);
}

void Fem1DElementBeam::initialize(const SurgSim::Math::OdeState& state)
{
	// Test the validity of the physical parameters
	FemElement::initialize(state);

	SURGSIM_ASSERT(m_radius > 0) << "Fem1DElementBeam radius should be positive and non-zero.  Did you call "
									"setCrossSectionCircular(radius) ?";

	m_A = M_PI * (m_radius * m_radius);
	const double iz = M_PI * (m_radius * m_radius * m_radius * m_radius) / 4.0;
	m_J = iz + iz;

	// Store the rest state for this beam in m_x0
	getSubVector(state.getPositions(), m_nodeIds, 6, &m_x0);

	m_restLength = (m_x0.segment<3>(6) - m_x0.segment<3>(0)).norm();
	SURGSIM_ASSERT(m_restLength > 0) << "Fem1DElementBeam rest length is zero (degenerate beam)";

	computeInitialRotation(state);

	// Pre-compute the mass and stiffness matrix
	computeMass(state);
	computeStiffness(state);
}

const SurgSim::Math::Matrix33d& Fem1DElementBeam::getInitialRotation() const
{
	return m_R0;
}

void Fem1DElementBeam::computeMass(const SurgSim::Math::OdeState& state)
{
	double& L = m_restLength;
	double L2 = L * L;
	double AL = m_A * L;
	double AL2 = AL * L;
	double m = AL * m_rho;

	const double Iz = M_PI * (m_radius * m_radius * m_radius * m_radius) / 4.0;
	const double Iy = Iz;

	m_MLocal.setZero();

	// From pg 294 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:

	// Mass matrix for node0 / node0
	m_MLocal(0, 0)   =  1.0 / 3.0;
	m_MLocal(1, 1)   =  13.0 / 35.0 + 6.0 * Iz / (5.0 * AL2);
	m_MLocal(1, 5)   =  11.0 * L / 210.0 + Iz / (10.0 * AL);
	m_MLocal(2, 2)   =  13.0 / 35.0 + 6.0 * Iy / (5.0 * AL2);
	m_MLocal(2, 4)   = -11.0 * L / 210.0 - Iy / (10.0 * AL);
	m_MLocal(3, 3)   =  m_J / (3.0 * m_A);
	m_MLocal(4, 2)   = -11.0 * L / 210.0 - Iy / (10.0 * AL);
	m_MLocal(4, 4)   =  L2 / 105.0 + 2.0 * Iy / (15.0 * m_A);
	m_MLocal(5, 1)   =  11.0 * L / 210.0 + Iz / (10.0 * AL);
	m_MLocal(5, 5)   =  L2 / 105.0 + 2.0 * Iz / (15.0 * m_A);

	// Mass matrix for node1 / node1
	m_MLocal(6, 6)   =  1.0 / 3.0;
	m_MLocal(7, 7)   =  13.0 / 35.0 + 6.0 * Iz / (5.0 * AL2);
	m_MLocal(7, 11)  = -11.0 * L / 210.0 - Iz / (10.0 * AL);
	m_MLocal(8, 8)   =  13.0 / 35.0 + 6.0 * Iy / (5.0 * AL2);
	m_MLocal(8, 10)  =  11.0 * L / 210.0 + Iy / (10.0 * AL);
	m_MLocal(9, 9)   =  m_J / (3.0 * m_A);
	m_MLocal(10, 8)  =  11.0 * L / 210.0 + Iy / (10.0 * AL);
	m_MLocal(10, 10) =  L2 / 105.0 + 2.0 * Iy / (15.0 * m_A);
	m_MLocal(11, 7)  = -11.0 * L / 210.0 - Iz / (10.0 * AL);
	m_MLocal(11, 11) =  L2 / 105.0 + 2.0 * Iz / (15.0 * m_A);

	// Mass matrix for node1 / node0
	m_MLocal(6, 0)   =  1.0 / 6.0;
	m_MLocal(7, 1)   =  9.0 / 70.0 - 6.0 * Iz / (5.0 * AL2);
	m_MLocal(7, 5)   =  13.0 * L / 420.0 - Iz / (10.0 * AL);
	m_MLocal(8, 2)   =  9.0 / 70.0 - 6.0 * Iy / (5.0 * AL2);
	m_MLocal(8, 4)   = -13.0 * L / 420.0 + Iy / (10.0 * AL);
	m_MLocal(9, 3)   =  m_J / (6.0 * m_A);
	m_MLocal(10, 2)  =  13.0 * L / 420.0 - Iy / (10.0 * AL);
	m_MLocal(10, 4)  = -L2 / 140.0 - Iy / (30.0 * m_A);
	m_MLocal(11, 1)  = -13.0 * L / 420.0 + Iz / (10.0 * AL);
	m_MLocal(11, 5)  = -L2 / 140.0 - Iz / (30.0 * m_A);

	// Mass matrix for node0 / node1
	m_MLocal(0, 6)   =  1.0 / 6.0;
	m_MLocal(1, 7)   =  9.0 / 70.0 - 6.0 * Iz / (5.0 * AL2);
	m_MLocal(1, 11)  = -13.0 * L / 420.0 + Iz / (10.0 * AL);
	m_MLocal(2, 8)   =  9.0 / 70.0 - 6.0 * Iy / (5.0 * AL2);
	m_MLocal(2, 10)  =  13.0 * L / 420.0 - Iy / (10.0 * AL);
	m_MLocal(3, 9)   =  m_J / (6.0 * m_A);
	m_MLocal(4, 8)   = -13.0 * L / 420.0 + Iy / (10.0 * AL);
	m_MLocal(4, 10)  = -L2 / 140.0 - Iy / (30.0 * m_A);
	m_MLocal(5, 7)   =  13.0 * L / 420.0 - Iz / (10.0 * AL);
	m_MLocal(5, 11)  = -L2 / 140.0 - Iz / (30.0 * m_A);

	m_MLocal *= m;

	// Transformation Local -> Global
	Eigen::Matrix<double, 12, 12> r0;
	r0.setZero();
	setSubMatrix(m_R0, 0, 0, 3, 3, &r0);
	setSubMatrix(m_R0, 1, 1, 3, 3, &r0);
	setSubMatrix(m_R0, 2, 2, 3, 3, &r0);
	setSubMatrix(m_R0, 3, 3, 3, 3, &r0);
	m_M = r0 * m_MLocal * r0.transpose();
}

void Fem1DElementBeam::computeStiffness(const SurgSim::Math::OdeState& state)
{
	double& L = m_restLength;
	double L2 = L * L;
	double L3 = L2 * L;

	/// Cross sectional moment of inertia
	const double Iz = M_PI * (m_radius * m_radius * m_radius * m_radius) / 4.0;
	const double Iy = Iz;

	// General expression for shear modulus in terms of Young's modulus and Poisson's ratio
	const double g = m_E / (2.0 * (1.0 + m_nu));

	/// The shear area in the y and z directions (=0 => no shear) http://en.wikipedia.org/wiki/Timoshenko_beam_theory
	double asy = 0.0;
	double asz = 0.0;

	/// Shear deformation parameters
	double phi_y = 0.0;
	double phi_z = 0.0;
	if (m_haveShear)
	{
		// Special values for solid cross sections.
		asy = m_A * 5.0 / 6.0;
		asz = m_A * 5.0 / 6.0;
		// From pg 80 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:
		phi_y = 12.0 * m_E * Iz / (g * asy * L2);
		phi_z = 12.0 * m_E * Iy / (g * asz * L2);
	}

	m_KLocal.setZero();

	// From pg 79 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:

	// Stiffness matrix node 1 / node 1
	m_KLocal(0, 0)   = m_E * m_A / L;
	m_KLocal(1, 1)   = 12 * m_E * Iz / (L3 * (1 + phi_y));
	m_KLocal(2, 2)   = 12 * m_E * Iy / (L3 * (1 + phi_z));
	m_KLocal(3, 3) = g * m_J / L;
	m_KLocal(4, 4)   = (4 + phi_z) * m_E * Iy / (L * (1 + phi_z));
	m_KLocal(4, 2)   = -6 * m_E * Iy / (L2 * (1 + phi_z)); // Symmetric
	m_KLocal(2, 4)   = m_KLocal(4, 2); // Symmetric
	m_KLocal(5, 5)   = (4 + phi_y) * m_E * Iz / (L * (1 + phi_y));
	m_KLocal(5, 1)   =  6 * m_E * Iz / (L2 * (1 + phi_y)); // Symmetric
	m_KLocal(1, 5)   = m_KLocal(5, 1); // Symmetric

	// Stiffness matrix node 2 / node 2
	m_KLocal(6, 6)   = m_E * m_A / L;
	m_KLocal(7, 7)   = 12 * m_E * Iz / (L3 * (1 + phi_y));
	m_KLocal(8, 8)   = 12 * m_E * Iy / (L3 * (1 + phi_z));
	m_KLocal(9, 9) = g * m_J / L;
	m_KLocal(10, 10) = (4 + phi_z) * m_E * Iy / (L * (1 + phi_z));
	m_KLocal(10, 8)  =  6 * m_E * Iy / (L2 * (1 + phi_z)); // Symmetric
	m_KLocal(8, 10)  = m_KLocal(10, 8); // Symmetric
	m_KLocal(11, 11) = (4 + phi_y) * m_E * Iz / (L * (1 + phi_y));
	m_KLocal(11, 7)  = -6 * m_E * Iz / (L2 * (1 + phi_y)); // Symmetric
	m_KLocal(7, 11)  = m_KLocal(11, 7); // Symmetric

	// Stiffness matrix node 2 / node 1
	m_KLocal(6, 0)   = -m_E * m_A / L;
	m_KLocal(7, 1)   = -12 * m_E * Iz / (L3 * (1 + phi_y));
	m_KLocal(8, 2)   = -12 * m_E * Iy / (L3 * (1 + phi_z));
	m_KLocal(9, 3) = -g * m_J / L;
	m_KLocal(10, 4)  = (2 - phi_z) * m_E * Iy / (L * (1 + phi_z));
	m_KLocal(10, 2)  = -6 * m_E * Iy / (L2 * (1 + phi_z)); // Anti-symmetric
	m_KLocal(8, 4)   = -m_KLocal(10, 2);					   // Anti-symmetric
	m_KLocal(11, 5)  = (2 - phi_y) * m_E * Iz / (L * (1 + phi_y));
	m_KLocal(11, 1)  =  6 * m_E * Iz / (L2 * (1 + phi_y)); // Anti-symmetric
	m_KLocal(7, 5)   = -m_KLocal(11, 1);					   // Anti-symmetric

	// Stiffness matrix node 1 / node 2 (symmetric of node 2 / node 1)
	m_KLocal(0, 6)   = m_KLocal(6, 0);
	m_KLocal(1, 7)   = m_KLocal(7, 1);
	m_KLocal(2, 8)   = m_KLocal(8, 2);
	m_KLocal(3, 9)   = m_KLocal(9, 3);
	m_KLocal(4, 10)  = m_KLocal(10, 4);
	m_KLocal(2, 10)  = m_KLocal(10, 2);
	m_KLocal(4, 8)   = m_KLocal(8, 4);
	m_KLocal(5, 11)  = m_KLocal(11, 5);
	m_KLocal(1, 11)  = m_KLocal(11, 1);
	m_KLocal(5, 7)   = m_KLocal(7, 5);

	// Transformation Local -> Global
	Eigen::Matrix<double, 12, 12> r0;
	r0.setZero();
	setSubMatrix(m_R0, 0, 0, 3, 3, &r0);
	setSubMatrix(m_R0, 1, 1, 3, 3, &r0);
	setSubMatrix(m_R0, 2, 2, 3, 3, &r0);
	setSubMatrix(m_R0, 3, 3, 3, 3, &r0);
	m_K = r0 * m_KLocal * r0.transpose();
}

void Fem1DElementBeam::computeInitialRotation(const SurgSim::Math::OdeState& state)
{
	// Build (i, j, k) an orthonormal frame
	const Vector3d A = state.getPosition(m_nodeIds[0]);
	const Vector3d B = state.getPosition(m_nodeIds[1]);
	Vector3d i = B - A;
	Vector3d j, k;

	SURGSIM_ASSERT(SurgSim::Math::buildOrthonormalBasis(&i, &j, &k))
			<< "Invalid beam formed by extremities A=(" << A.transpose() << ") B=(" << B.transpose() << ")";

	m_R0.col(0) = i;
	m_R0.col(1) = j;
	m_R0.col(2) = k;
}

SurgSim::Math::Vector Fem1DElementBeam::computeCartesianCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& naturalCoordinate) const
{
	SURGSIM_ASSERT(isValidCoordinate(naturalCoordinate)) << "naturalCoordinate must be normalized and length 2.";
	const Vector& positions = state.getPositions();
	return naturalCoordinate(0) * getSubVector(positions, m_nodeIds[0], getNumDofPerNode()).segment<3>(0) +
		naturalCoordinate(1) * getSubVector(positions, m_nodeIds[1], getNumDofPerNode()).segment<3>(0);
}

void Fem1DElementBeam::doUpdateFMDK(const Math::OdeState& state, int options)
{
	if (options & SurgSim::Math::OdeEquationUpdate::ODEEQUATIONUPDATE_F)
	{
		Eigen::Matrix<double, 12, 1> x;

		// K.U = F_ext
		// K.(x - x0) = F_ext
		// 0 = F_ext + F_int, with F_int = -K.(x - x0)
		getSubVector(state.getPositions(), m_nodeIds, 6, &x);
		m_f.noalias() = -m_K * (x - m_x0);
	}
}

SurgSim::Math::Vector Fem1DElementBeam::computeNaturalCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& cartesianCoordinate) const
{
	SURGSIM_FAILURE() << "Function " << __FUNCTION__ << " not yet implemented.";
	return SurgSim::Math::Vector3d::Zero();
}

} // namespace Physics

} // namespace SurgSim
