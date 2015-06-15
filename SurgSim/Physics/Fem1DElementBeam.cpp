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
	m_G = 0.0;
	m_restLength = 0.0;
	m_radius = 0.0;
	m_A = 0.0;
	m_haveShear = true;
	m_shearFactor = (5.0 / 8.0);
	m_Asy = 0.0;
	m_Asz = 0.0;
	m_Phi_y = 0.0;
	m_Phi_z = 0.0;
	m_Iy = 0.0;
	m_Iz = 0.0;
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
	m_Iz = M_PI * (m_radius * m_radius * m_radius * m_radius) / 4.0;
	m_Iy = m_Iz;
	m_J = m_Iz + m_Iy;

	// Store the rest state for this beam in m_x0
	getSubVector(state.getPositions(), m_nodeIds, 6, &m_x0);

	m_restLength = (m_x0.segment<3>(6) - m_x0.segment<3>(0)).norm();
	SURGSIM_ASSERT(m_restLength > 0) << "Fem1DElementBeam rest length is zero (degenerate beam)";

	computeInitialRotation(state);

	// Pre-compute the mass and stiffness matrix
	computeMass(state, &m_M);
	computeStiffness(state, &m_K);
}

void Fem1DElementBeam::addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale)
{
	Eigen::Matrix<double, 12, 1> x, f;

	// K.U = F_ext
	// K.(x - x0) = F_ext
	// 0 = F_ext + F_int, with F_int = -K.(x - x0)
	getSubVector(state.getPositions(), m_nodeIds, 6, &x);
	f = (-scale) * m_K * (x - m_x0);
	addSubVector(f, m_nodeIds, 6, F);
}

void Fem1DElementBeam::addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* M,
							   double scale)
{
	assembleMatrixBlocks(m_M * scale, m_nodeIds, 6, M, false);
}

void Fem1DElementBeam::addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* D,
								  double scale)
{
}

void Fem1DElementBeam::addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* K,
									double scale)
{
	assembleMatrixBlocks(m_K * scale, getNodeIds(), 6, K, false);
}

void Fem1DElementBeam::addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
							   SurgSim::Math::SparseMatrix* M, SurgSim::Math::SparseMatrix* D,
							   SurgSim::Math::SparseMatrix* K)
{
	// Assemble the mass matrix
	addMass(state, M);

	// No damping matrix as we are using linear elasticity (not visco-elasticity)

	// Assemble the stiffness matrix
	addStiffness(state, K);

	// Assemble the force vector
	addForce(state, F);
}

void Fem1DElementBeam::addMatVec(const SurgSim::Math::OdeState& state, double alphaM, double alphaD,
								 double alphaK, const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubVector;

	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 12, 1> extractedX;
	getSubVector(x, m_nodeIds, 6, &extractedX);

	// Adds the mass contribution
	if (alphaM != 0.0)
	{
		addSubVector(alphaM * (m_M * extractedX), m_nodeIds, 6, F);
	}

	// Adds the damping contribution (No damping)

	// Adds the stiffness contribution
	if (alphaK != 0.0)
	{
		addSubVector(alphaK * (m_K * extractedX), m_nodeIds, 6, F);
	}
}

void Fem1DElementBeam::computeMass(const SurgSim::Math::OdeState& state,
								   Eigen::Matrix<double, 12, 12>* M)
{
	double& L = m_restLength;
	double L2 = L * L;
	double AL = m_A * L;
	double AL2 = AL * L;
	double m = AL * m_rho;

	m_MLocal.setZero();

	// From pg 294 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:

	// Mass matrix for node0 / node0
	m_MLocal(0, 0)   =  1.0 / 3.0;
	m_MLocal(1, 1)   =  13.0 / 35.0 + 6.0 * m_Iz / (5.0 * AL2);
	m_MLocal(1, 5)   =  11.0 * L / 210.0 + m_Iz / (10.0 * AL);
	m_MLocal(2, 2)   =  13.0 / 35.0 + 6.0 * m_Iy / (5.0 * AL2);
	m_MLocal(2, 4)   = -11.0 * L / 210.0 - m_Iy / (10.0 * AL);
	m_MLocal(3, 3)   =  m_J / (3.0 * m_A);
	m_MLocal(4, 2)   = -11.0 * L / 210.0 - m_Iy / (10.0 * AL);
	m_MLocal(4, 4)   =  L2 / 105.0 + 2.0 * m_Iy / (15.0 * m_A);
	m_MLocal(5, 1)   =  11.0 * L / 210.0 + m_Iz / (10.0 * AL);
	m_MLocal(5, 5)   =  L2 / 105.0 + 2.0 * m_Iz / (15.0 * m_A);

	// Mass matrix for node1 / node1
	m_MLocal(6, 6)   =  1.0 / 3.0;
	m_MLocal(7, 7)   =  13.0 / 35.0 + 6.0 * m_Iz / (5.0 * AL2);
	m_MLocal(7, 11)  = -11.0 * L / 210.0 - m_Iz / (10.0 * AL);
	m_MLocal(8, 8)   =  13.0 / 35.0 + 6.0 * m_Iy / (5.0 * AL2);
	m_MLocal(8, 10)  =  11.0 * L / 210.0 + m_Iy / (10.0 * AL);
	m_MLocal(9, 9)   =  m_J / (3.0 * m_A);
	m_MLocal(10, 8)  =  11.0 * L / 210.0 + m_Iy / (10.0 * AL);
	m_MLocal(10, 10) =  L2 / 105.0 + 2.0 * m_Iy / (15.0 * m_A);
	m_MLocal(11, 7)  = -11.0 * L / 210.0 - m_Iz / (10.0 * AL);
	m_MLocal(11, 11) =  L2 / 105.0 + 2.0 * m_Iz / (15.0 * m_A);

	// Mass matrix for node1 / node0
	m_MLocal(6, 0)   =  1.0 / 6.0;
	m_MLocal(7, 1)   =  9.0 / 70.0 - 6.0 * m_Iz / (5.0 * AL2);
	m_MLocal(7, 5)   =  13.0 * L / 420.0 - m_Iz / (10.0 * AL);
	m_MLocal(8, 2)   =  9.0 / 70.0 - 6.0 * m_Iy / (5.0 * AL2);
	m_MLocal(8, 4)   = -13.0 * L / 420.0 + m_Iy / (10.0 * AL);
	m_MLocal(9, 3)   =  m_J / (6.0 * m_A);
	m_MLocal(10, 2)  =  13.0 * L / 420.0 - m_Iy / (10.0 * AL);
	m_MLocal(10, 4)  = -L2 / 140.0 - m_Iy / (30.0 * m_A);
	m_MLocal(11, 1)  = -13.0 * L / 420.0 + m_Iz / (10.0 * AL);
	m_MLocal(11, 5)  = -L2 / 140.0 - m_Iz / (30.0 * m_A);

	// Mass matrix for node0 / node1
	m_MLocal(0, 6)   =  1.0 / 6.0;
	m_MLocal(1, 7)   =  9.0 / 70.0 - 6.0 * m_Iz / (5.0 * AL2);
	m_MLocal(1, 11)  = -13.0 * L / 420.0 + m_Iz / (10.0 * AL);
	m_MLocal(2, 8)   =  9.0 / 70.0 - 6.0 * m_Iy / (5.0 * AL2);
	m_MLocal(2, 10)  =  13.0 * L / 420.0 - m_Iy / (10.0 * AL);
	m_MLocal(3, 9)   =  m_J / (6.0 * m_A);
	m_MLocal(4, 8)   = -13.0 * L / 420.0 + m_Iy / (10.0 * AL);
	m_MLocal(4, 10)  = -L2 / 140.0 - m_Iy / (30.0 * m_A);
	m_MLocal(5, 7)   =  13.0 * L / 420.0 - m_Iz / (10.0 * AL);
	m_MLocal(5, 11)  = -L2 / 140.0 - m_Iz / (30.0 * m_A);

	m_MLocal *= m;

	// Transformation Local -> Global
	m_M = m_R0 * m_MLocal * m_R0.transpose();
}

void Fem1DElementBeam::computeStiffness(const SurgSim::Math::OdeState& state,
										Eigen::Matrix<double, 12, 12>* k)
{
	double& L = m_restLength;
	double L2 = L * L;
	double L3 = L2 * L;

	// General expression for shear modulus in terms of Young's modulus and Poisson's ratio
	m_G = m_E / (2.0 * (1.0 + m_nu));

	if (m_haveShear)
	{
		// Special values for solid cross sections.
		m_Asy = m_A * 5.0 / 6.0;
		m_Asz = m_A * 5.0 / 6.0;
		// From pg 80 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:
		m_Phi_y = 12.0 * m_E * m_Iz / (m_G * m_Asy * L2);
		m_Phi_z = 12.0 * m_E * m_Iy / (m_G * m_Asz * L2);
	}
	else
	{
		m_Asy   = 0.0;
		m_Asz   = 0.0;
		m_Phi_y = 0.0;
		m_Phi_z = 0.0;
	}

	m_KLocal.setZero();

	// From pg 79 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:

	// Stiffness matrix node 1 / node 1
	m_KLocal(0, 0)   = m_E * m_A / L;
	m_KLocal(1, 1)   = 12 * m_E * m_Iz / (L3 * (1 + m_Phi_y));
	m_KLocal(2, 2)   = 12 * m_E * m_Iy / (L3 * (1 + m_Phi_z));
	m_KLocal(3, 3)   = m_G * m_J / L;
	m_KLocal(4, 4)   = (4 + m_Phi_z) * m_E * m_Iy / (L * (1 + m_Phi_z));
	m_KLocal(4, 2)   = -6 * m_E * m_Iy / (L2 * (1 + m_Phi_z)); // Symmetric
	m_KLocal(2, 4)   = -6 * m_E * m_Iy / (L2 * (1 + m_Phi_z)); // Symmetric
	m_KLocal(5, 5)   = (4 + m_Phi_y) * m_E * m_Iz / (L * (1 + m_Phi_y));
	m_KLocal(5, 1)   =  6 * m_E * m_Iz / (L2 * (1 + m_Phi_y)); // Symmetric
	m_KLocal(1, 5)   =  6 * m_E * m_Iz / (L2 * (1 + m_Phi_y)); // Symmetric

	// Stiffness matrix node 2 / node 2
	m_KLocal(6, 6)   = m_E * m_A / L;
	m_KLocal(7, 7)   = 12 * m_E * m_Iz / (L3 * (1 + m_Phi_y));
	m_KLocal(8, 8)   = 12 * m_E * m_Iy / (L3 * (1 + m_Phi_z));
	m_KLocal(9, 9)   = m_G * m_J / L;
	m_KLocal(10, 10) = (4 + m_Phi_z) * m_E * m_Iy / (L * (1 + m_Phi_z));
	m_KLocal(10, 8)  =  6 * m_E * m_Iy / (L2 * (1 + m_Phi_z)); // Symmetric
	m_KLocal(8, 10)  =  6 * m_E * m_Iy / (L2 * (1 + m_Phi_z)); // Symmetric
	m_KLocal(11, 11) = (4 + m_Phi_y) * m_E * m_Iz / (L * (1 + m_Phi_y));
	m_KLocal(11, 7)  = -6 * m_E * m_Iz / (L2 * (1 + m_Phi_y)); // Symmetric
	m_KLocal(7, 11)  = -6 * m_E * m_Iz / (L2 * (1 + m_Phi_y)); // Symmetric

	// Stiffness matrix node 2 / node 1
	m_KLocal(6, 0)   = -m_E * m_A / L;
	m_KLocal(7, 1)   = -12 * m_E * m_Iz / (L3 * (1 + m_Phi_y));
	m_KLocal(8, 2)   = -12 * m_E * m_Iy / (L3 * (1 + m_Phi_z));
	m_KLocal(9, 3)   = -m_G * m_J / L;
	m_KLocal(10, 4)  = (2 - m_Phi_z) * m_E * m_Iy / (L * (1 + m_Phi_z));
	m_KLocal(10, 2)  = -6 * m_E * m_Iy / (L2 * (1 + m_Phi_z)); // Anti-symmetric
	m_KLocal(8, 4)   = -m_KLocal(10, 2);					   // Anti-symmetric
	m_KLocal(11, 5)  = (2 - m_Phi_y) * m_E * m_Iz / (L * (1 + m_Phi_y));
	m_KLocal(11, 1)  =  6 * m_E * m_Iz / (L2 * (1 + m_Phi_y)); // Anti-symmetric
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
	m_K = m_R0 * m_KLocal * m_R0.transpose();
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

	// Set up a temporary 3x3 initial rotation matrix
	SurgSim::Math::Matrix33d rotation3x3;
	rotation3x3.col(0) = i;
	rotation3x3.col(1) = j;
	rotation3x3.col(2) = k;

	// Set up the 12x12 initial rotation matrix
	m_R0.setZero();
	setSubMatrix(rotation3x3, 0, 0, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 1, 1, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 2, 2, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 3, 3, 3, 3, &m_R0);
}

SurgSim::Math::Vector Fem1DElementBeam::computeCartesianCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& naturalCoordinate) const
{
	SURGSIM_ASSERT(isValidCoordinate(naturalCoordinate)) << "naturalCoordinate must be normalized and length 2.";

	Vector3d cartesianCoordinate(0.0, 0.0, 0.0);

	const Vector& positions = state.getPositions();

	for (int i = 0; i < 2; i++)
	{
		cartesianCoordinate += naturalCoordinate(i) * getSubVector(positions, m_nodeIds[i], 6).segment<3>(0);
	}

	return cartesianCoordinate;
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
