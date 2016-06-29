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
#include "SurgSim/Physics/Fem1DElementTimoshenkoBeam.h"

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
SURGSIM_REGISTER(SurgSim::Physics::FemElement, SurgSim::Physics::Fem1DElementTimoshenkoBeam, Fem1DElementTimoshenkoBeam)

Fem1DElementTimoshenkoBeam::Fem1DElementTimoshenkoBeam()
{
	initializeMembers();
}

Fem1DElementTimoshenkoBeam::Fem1DElementTimoshenkoBeam(std::array<size_t, 2> nodeIds)
{
	initializeMembers();
	m_nodeIds.assign(nodeIds.cbegin(), nodeIds.cend());
}

Fem1DElementTimoshenkoBeam::Fem1DElementTimoshenkoBeam(std::shared_ptr<FemElementStructs::FemElementParameter> elementData)
{
	initializeMembers();
	auto element1DData = std::dynamic_pointer_cast<FemElementStructs::FemElement1DParameter>(elementData);
	SURGSIM_ASSERT(element1DData != nullptr) << "Incorrect struct type passed";
	SURGSIM_ASSERT(element1DData->nodeIds.size() == 2) << "Incorrect number of nodes for a Fem1D Beam";
	m_nodeIds.assign(element1DData->nodeIds.begin(), element1DData->nodeIds.end());
	setRadius(element1DData->radius);
	setMassDensity(element1DData->massDensity);
	setPoissonRatio(element1DData->poissonRatio);
	setYoungModulus(element1DData->youngModulus);
}

void Fem1DElementTimoshenkoBeam::setRadius(double radius)
{
	SURGSIM_ASSERT(radius != 0.0) << "The beam radius cannot be set to 0";
	SURGSIM_ASSERT(radius > 0.0) << "The beam radius cannot be negative (trying to set it to " << radius << ")";

	m_radius = radius;
}

double Fem1DElementTimoshenkoBeam::getRadius() const
{
	return m_radius;
}

double Fem1DElementTimoshenkoBeam::getVolume(const SurgSim::Math::OdeState& state) const
{
	const Vector3d A = state.getPosition(m_nodeIds[0]);
	const Vector3d B = state.getPosition(m_nodeIds[1]);

	return m_A * (B - A).norm();
}

void Fem1DElementTimoshenkoBeam::initializeMembers()
{
	m_G = 0.0;
	m_restLength = 0.0;
	m_radius = 0.0;
	m_A = 0.0;
	m_shearFactor = 1.0;// (5.0 / 8.0);
	m_Iy = 0.0;
	m_Iz = 0.0;
	m_J = 0.0;

	// 6 dof per node (x, y, z, thetaX, thetaY, thetaZ)
	setNumDofPerNode(6);
	printf("Timoshenko beam\n");
}

void Fem1DElementTimoshenkoBeam::initialize(const SurgSim::Math::OdeState& state)
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
	computeMass(state);
	computeStiffness(state);
}

void Fem1DElementTimoshenkoBeam::computeMass(const SurgSim::Math::OdeState& state)
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

void Fem1DElementTimoshenkoBeam::computeStiffness(const SurgSim::Math::OdeState& state)
{
	double& L = m_restLength;
	double L2 = L * L;
	double L3 = L2 * L;

	// General expression for shear modulus in terms of Young's modulus and Poisson's ratio
	m_G = m_E / (2.0 * (1.0 + m_nu));

	double EA = m_E * m_A;
	double EIy = m_E * m_Iy;
	double EIz = m_E * m_Iz;
	double kGA = m_shearFactor * m_G * m_A;
	double kGJ = m_shearFactor * m_G * m_J;
	double kGAL2 = kGA * L2;

	m_alphay = 12.0 * EIy / kGAL2;
	m_alphaz = 12.0 * EIz / kGAL2;
	m_betay = 1.0 / (1.0 - m_alphay);
	m_betaz = 1.0 / (1.0 - m_alphaz);

	const double& ay = m_alphay;
	const double& az = m_alphaz;
	const double& by = m_betay;
	const double& bz = m_betaz;
	double ay2 = ay * ay;
	double az2 = az * az;
	double by2 = by * by;
	double bz2 = bz * bz;

	m_KLocal.setZero();

	// From pg 79 of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:

	// Column 0
	m_KLocal(0, 0) = EA / L;
	m_KLocal(6, 0) = -m_KLocal(0, 0);
	m_KLocal(0, 6) = m_KLocal(6, 0); // symmetry

	// Column 1
	m_KLocal(1, 1) = 12.0 * by2 * EIy / L3 + by2 * ay2 * kGA / L;
	m_KLocal(4, 1) = 6.0 * by2 * EIy / L2 + 0.5 * by2 * ay2 * kGA;
	m_KLocal(7, 1) = -12.0 * by2 * EIy / L3;
	m_KLocal(11, 1) = m_KLocal(4, 1);
	m_KLocal(1, 4) = m_KLocal(4, 1); // symmetry
	m_KLocal(1, 7) = m_KLocal(7, 1); // symmetry
	m_KLocal(1, 11) = m_KLocal(11, 1); // symmetry

	// Column 2
	m_KLocal(2, 2) = 12.0 * bz2 * EIz / L3 + 1.0 / 5.0 * bz2 * kGA * (24.0 - 20.0 * az + 5.0 * az2) / L;
	m_KLocal(5, 2) = 6.0 * bz2 * EIz / L2 + 1.0 / 2.0 * bz2 * az * kGA * (-2.0 + az);
	m_KLocal(7, 2) = -by * ay * bz * (-2.0 + az) * kGA / L;
	m_KLocal(8, 2) = -m_KLocal(2, 2);
	m_KLocal(11, 2) = 6.0 * bz2 * EIz / L2 - 1.0 / 10.0 * bz2 * kGA * (-4.0 - 10.0 * az + 5.0 * az2);
	m_KLocal(2, 5) = m_KLocal(5, 2); // symmetry
	m_KLocal(2, 7) = m_KLocal(7, 2); // symmetry
	m_KLocal(2, 8) = m_KLocal(8, 2); // symmetry
	m_KLocal(2, 11) = m_KLocal(11, 2); // symmetry

	// Column 3
	m_KLocal(3, 3) = kGJ / L;
	m_KLocal(9, 3) = -m_KLocal(3, 3);
	m_KLocal(3, 9) = m_KLocal(9, 3); // symmetry

	// Column 4
	m_KLocal(4, 4) = by2 * (4.0 - 2.0 * ay + ay2) * EIy / L + 1.0 / 4.0 * by2 * ay2 * L * kGA;
	m_KLocal(7, 4) = -6.0 * by2 * EIy / L2;
	m_KLocal(10, 4) = -by2 * (-2.0 - 2.0 * ay + ay2) * EIy / L + 1.0 / 4.0 * by2 * ay2 * L * kGA;
	m_KLocal(4, 7) = m_KLocal(7, 4); // symmetry
	m_KLocal(4, 10) = m_KLocal(10, 4); // symmetry

	// Column 5
	m_KLocal(5, 5) = bz2 * (4.0 - 2.0 * az + az2) * EIz / L + 1.0 / 4.0 * bz2 * az2 * L * kGA;
	m_KLocal(7, 5) = -1.0 / 2.0 * by * ay * bz * az * kGA;
	m_KLocal(8, 5) = -6.0 * bz2 * EIz / L2 - 1.0 / 2.0 * bz2 * az * (-2.0 + az) * kGA;
	m_KLocal(11, 5) = -bz2 * (-2.0 - 2.0 * az + az2) * EIz / L - 1.0 / 4.0 * bz2 * az2 * L * kGA;
	m_KLocal(5, 7) = m_KLocal(7, 5); // symmetry
	m_KLocal(5, 8) = m_KLocal(8, 5); // symmetry
	m_KLocal(5, 11) = m_KLocal(11, 5); // symmetry

	// Column 6
	m_KLocal(6, 6) = EA / L;

	// Column 7
	m_KLocal(7, 7) = 12.0 * by2 * EIy / L3 + by2 * ay2 * kGA / L;
	m_KLocal(8, 7) = by * ay * bz * (-2.0 + az) * kGA / L;
	m_KLocal(10, 7) = -6.0 * by2 * EIy / L2;
	m_KLocal(11, 7) = 1.0 / 2.0 * by * ay * bz * az * kGA;
	m_KLocal(7, 8) = m_KLocal(8, 7); // symmetry
	m_KLocal(7, 10) = m_KLocal(10, 7); // symmetry
	m_KLocal(7, 11) = m_KLocal(11, 7); // symmetry

	// Column 8
	m_KLocal(8, 8) = 12.0 * bz2 * EIz / L3 + 1.0 / 5.0 * bz2 * (24.0 - 20.0 * az + 5.0 * az2) * kGA / L;
	m_KLocal(11, 8) = -6.0 * bz2 * EIz / L2 + 1.0 / 10.0 * bz2 * (-4.0 - 10.0 * az + 5.0 * az2) * kGA;
	m_KLocal(8, 11) = m_KLocal(11, 8); // symmetry

	// Column 9
	m_KLocal(9, 9) = kGJ / L;

	// Column 10
	m_KLocal(10, 10) = by2 * (4.0 - 2.0 * ay + ay2) * EIy / L + 1.0 / 4.0 * by2 * ay2 * L * kGA;

	// Column 11
	m_KLocal(11, 11) = bz2 * (4.0 - 2.0 * az + az2) * EIz / L + 1.0 / 60.0 * bz2 * L * (32.0 - 40.0 * az + 35.0 * az2) * kGA;

	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			if (m_KLocal(i, j) != m_KLocal(j, i))
				printf("m_KLocal(% 2d, % 2d)= %g    m_KLocal(% 2d, % 2d)= %g\n",i,j, m_KLocal(i, j), j, i, m_KLocal(j, i));
		}
	}

	// Transformation Local -> Global
	m_K = m_R0 * m_KLocal * m_R0.transpose();
}

void Fem1DElementTimoshenkoBeam::computeInitialRotation(const SurgSim::Math::OdeState& state)
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

SurgSim::Math::Vector Fem1DElementTimoshenkoBeam::computeCartesianCoordinate(
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

void Fem1DElementTimoshenkoBeam::doUpdateFMDK(const Math::OdeState& state, int options)
{
	if (options & SurgSim::Math::OdeEquationUpdate::ODEEQUATIONUPDATE_F)
	{
		Eigen::Matrix<double, 12, 1> x;

		// K.U = F_ext
		// K.(x - x0) = F_ext
		// 0 = F_ext + F_int, with F_int = -K.(x - x0)
		getSubVector(state.getPositions(), m_nodeIds, 6, &x);
		m_f = -m_K * (x - m_x0);
	}
}

SurgSim::Math::Vector Fem1DElementTimoshenkoBeam::computeNaturalCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector& cartesianCoordinate) const
{
	SURGSIM_FAILURE() << "Function " << __FUNCTION__ << " not yet implemented.";
	return SurgSim::Math::Vector3d::Zero();
}

double Fem1DElementTimoshenkoBeam::N1(double x) const
{
	double xi = x / m_restLength;
	return 1 - xi;
}

double Fem1DElementTimoshenkoBeam::N2(double x) const
{
	double xi = x / m_restLength;
	return xi;
}

double Fem1DElementTimoshenkoBeam::Hv1(double x) const
{
	double xi = x / m_restLength;
	return m_betay * (2*xi*xi*xi - 3*xi*xi + m_alphay*xi + 1 - m_alphay);
}

double Fem1DElementTimoshenkoBeam::Hv2(double x) const
{
	double xi = x / m_restLength;
	return m_betay * (-2*xi*xi*xi + 3*xi*xi - m_alphay*xi);
}

double Fem1DElementTimoshenkoBeam::Hw1(double x) const
{
	double xi = x / m_restLength;
	return m_betaz * (2 * xi*xi*xi - 3 * xi*xi + m_alphaz*xi + 1 - m_alphaz);
}

double Fem1DElementTimoshenkoBeam::Hw2(double x) const
{
	double xi = x / m_restLength;
	return m_betaz * (-2 * xi*xi*xi + 3 * xi*xi - m_alphaz*xi);
}

double Fem1DElementTimoshenkoBeam::Htheta1(double x) const
{
	double xi = x / m_restLength;
	return m_restLength * m_betay * (xi*xi*xi + (0.5*m_alphay - 2)*xi*xi + (1-0.5*m_alphay)*xi);
}

double Fem1DElementTimoshenkoBeam::Htheta2(double x) const
{
	double xi = x / m_restLength;
	return m_restLength * m_betay * (xi*xi*xi - (1 + 0.5*m_alphay)*xi*xi + (0.5*m_alphay)*xi);
}


double Fem1DElementTimoshenkoBeam::Hpsi1(double x) const
{
	double xi = x / m_restLength;
	return m_restLength * m_betaz * (xi*xi*xi + (0.5*m_alphaz - 2)*xi*xi + (1 - 0.5*m_alphaz)*xi);
}

double Fem1DElementTimoshenkoBeam::Hpsi2(double x) const
{
	double xi = x / m_restLength;
	return m_restLength * m_betaz * (xi*xi*xi - (1+0.5*m_alphaz)*xi*xi + (0.5*m_alphaz)*xi);
}

double Fem1DElementTimoshenkoBeam::Gv1(double x) const
{
	double xi = x / m_restLength;
	return 6 * m_betay / m_restLength * (xi*xi - xi);
}

double Fem1DElementTimoshenkoBeam::Gv2(double x) const
{
	double xi = x / m_restLength;
	return 6 * m_betay / m_restLength * (-xi*xi + xi);
}

double Fem1DElementTimoshenkoBeam::Gw1(double x) const
{
	double xi = x / m_restLength;
	return 6 * m_betaz / m_restLength * (xi*xi - xi);
}

double Fem1DElementTimoshenkoBeam::Gw2(double x) const
{
	double xi = x / m_restLength;
	return 6 * m_betaz / m_restLength * (-xi*xi + xi);
}

double Fem1DElementTimoshenkoBeam::Gtheta1(double x) const
{
	double xi = x / m_restLength;
	return m_betay * (3*xi*xi + (m_alphay - 4)*xi + 1 - m_alphay);
}

double Fem1DElementTimoshenkoBeam::Gtheta2(double x) const
{
	double xi = x / m_restLength;
	return m_betay * (3 * xi*xi - (m_alphay + 2)*xi);
}

double Fem1DElementTimoshenkoBeam::Gpsi1(double x) const
{
	double xi = x / m_restLength;
	return m_betaz * (3 * xi*xi + (m_alphaz - 4)*xi + 1 - m_alphaz);
}

double Fem1DElementTimoshenkoBeam::Gpsi2(double x) const
{
	double xi = x / m_restLength;
	return m_betaz * (3 * xi*xi - (m_alphaz + 2)*xi);
}

// Derived shape functions
double Fem1DElementTimoshenkoBeam::N1prime(double x) const
{
	return -1 / m_restLength;
}

double Fem1DElementTimoshenkoBeam::N2prime(double x) const
{
	return 1 / m_restLength;
}

double Fem1DElementTimoshenkoBeam::Hv1prime(double x) const
{
	double L = m_restLength;
	return m_betay * (6 * x*x/(L*L*L) - 6 * x/(L*L) + m_alphay/L);
}

double Fem1DElementTimoshenkoBeam::Hv2prime(double x) const
{
	double L = m_restLength;
	return m_betay * (-6 * x*x/(L*L*L) + 6 * x/(L*L) - m_alphay/L);
}

double Fem1DElementTimoshenkoBeam::Hw1prime(double x) const
{
	double L = m_restLength;
	return m_betaz * (6 * x*x/(L*L*L) - 6 * x/(L*L) + m_alphaz/L);
}

double Fem1DElementTimoshenkoBeam::Hw2prime(double x) const
{
	double L = m_restLength;
	return m_betaz * (-6 * x*x/(L*L*L) + 6 * x/(L*L) - m_alphaz/L);
}

double Fem1DElementTimoshenkoBeam::Htheta1prime(double x) const
{
	double L = m_restLength;
	return L * m_betay * (3*x*x/(L*L*L) + (0.5*m_alphay - 2)*2*x/(L*L) + (1 - 0.5*m_alphay)/L);
}

double Fem1DElementTimoshenkoBeam::Htheta2prime(double x) const
{
	double L = m_restLength;
	return L * m_betay * (3*x*x/(L*L*L) - (1 + 0.5*m_alphay)*2*x/(L*L) + (0.5*m_alphay)/L);
}


double Fem1DElementTimoshenkoBeam::Hpsi1prime(double x) const
{
	double L = m_restLength;
	return L * m_betaz * (3*x*x/(L*L*L) + (0.5*m_alphaz - 2)*2*x/(L*L) + (1 - 0.5*m_alphaz)/L);
}

double Fem1DElementTimoshenkoBeam::Hpsi2prime(double x) const
{
	double L = m_restLength;
	return L * m_betaz * (3*x*x/(L*L*L) - (1 + 0.5*m_alphaz)*2*x/(L*L) + (0.5*m_alphaz)/L);
}

double Fem1DElementTimoshenkoBeam::Gv1prime(double x) const
{
	double L = m_restLength;
	return 6 * m_betay / L * (2*x/(L*L) - 1/L);
}

double Fem1DElementTimoshenkoBeam::Gv2prime(double x) const
{
	double L = m_restLength;
	return 6 * m_betay / L * (-2*x/(L*L) + 1/L);
}

double Fem1DElementTimoshenkoBeam::Gw1prime(double x) const
{
	double L = m_restLength;
	return 6 * m_betaz / L * (2*x/(L*L) - 1/L);
}

double Fem1DElementTimoshenkoBeam::Gw2prime(double x) const
{
	double L = m_restLength;
	return 6 * m_betaz / L * (-2*x/(L*L) + 1/L);
}

double Fem1DElementTimoshenkoBeam::Gtheta1prime(double x) const
{
	double L = m_restLength;
	return m_betay * (6 * x/(L*L) + (m_alphay - 4)/L);
}

double Fem1DElementTimoshenkoBeam::Gtheta2prime(double x) const
{
	double L = m_restLength;
	return m_betay * (6 * x/(L*L) - (m_alphay + 2)/L);
}

double Fem1DElementTimoshenkoBeam::Gpsi1prime(double x) const
{
	double L = m_restLength;
	return m_betaz * (6 * x/(L*L) + (m_alphaz - 4)/L);
}

double Fem1DElementTimoshenkoBeam::Gpsi2prime(double x) const
{
	double L = m_restLength;
	return m_betaz * (6 * x/(L*L) - (m_alphaz + 2)/L);
}

Eigen::Matrix<double, 6, 12> Fem1DElementTimoshenkoBeam::B(double x) const
{
	Eigen::Matrix<double, 6, 12> b = Eigen::Matrix<double, 6, 12>::Zero();

	// Axial strain
	b(0, 0) = N1prime(x);
	b(0, 6) = N2prime(x);

	// Curvature
	b(1, 1) = -Gv1prime(x);
	b(1, 4) = -Gtheta1prime(x);
	b(1, 7) = -Gv2prime(x);
	b(1, 10) = -Gtheta2prime(x);

	// Curvature
	b(2, 2) = Gw1prime(x);
	b(2, 5) = Gpsi1prime(x);
	b(2, 8) = Gw2prime(x);
	b(2, 11) = Gpsi2prime(x);

	// Shear
	b(3, 2) = Hv1prime(x) - Gv1(x);
	b(3, 4) = Htheta1prime(x) - Gtheta1(x);
	b(3, 7) = Hv2prime(x) - Gv2(x);
	b(3, 10) = Htheta2prime(x) - Gtheta2(x);

	// Shear
	b(4, 2) = Hv1prime(x) - Gv1(x);
	b(4, 4) = Htheta1prime(x) - Gtheta1(x);
	b(4, 7) = Hv2prime(x) - Gv2(x);
	b(4, 10) = Htheta2prime(x) - Gtheta2(x);

	return b
}

} // namespace Physics

} // namespace SurgSim
