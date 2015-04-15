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

#include <gtest/gtest.h>

#include <memory>
#include <array>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/GaussLegendreQuadrature.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::SparseMatrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem1DElementBeam;

namespace
{
const double epsilon = 1e-9;
};

class MockFem1DElement : public Fem1DElementBeam
{
public:
	explicit MockFem1DElement(std::array<size_t, 2> nodeIds)
		: Fem1DElementBeam(nodeIds)
	{
	}

	const Eigen::Matrix<double, 12, 12>& getInitialRotation() const
	{
		return m_R0;
	}

	double getRestLength() const
	{
		return m_restLength;
	}
};

void computeShapeFunction(double xi, double eta, double zeta, double L, Eigen::Ref<Eigen::Matrix<double, 3, 12>> N)
{
	// Shape function for beam from "Theory of Matrix Structural Analysis", Przemieniecki.  Eqns 11.30.
	// Note xi = x/l, eta = y/l, zeta = z/l

	double xi2 = xi * xi;
	double xi3 = xi2 * xi;

	// Node 1
	N(0, 0) = 1.0 - xi;
	N(0, 1) = 6.0 * (xi - xi2) * eta;
	N(0, 2) = 6.0 * (xi - xi2) * zeta;
	N(0, 3) = 0.0;
	N(0, 4) = (1.0 - 4.0 * xi + 3.0 * xi2) * L * zeta;
	N(0, 5) = (-1.0 + 4.0 * xi - 3.0 * xi2) * L * eta;

	N(1, 0) = 0.0;
	N(1, 1) = 1.0 - 3.0 * xi2 + 2.0 * xi3;
	N(1, 2) = 0.0;
	N(1, 3) = -(1.0 - xi) * L * zeta;
	N(1, 4) = 0.0;
	N(1, 5) = (xi - 2.0 * xi2 + xi3) * L;

	N(2, 0) = 0.0;
	N(2, 1) = 0.0;
	N(2, 2) = 1.0 - 3.0 * xi2 + 2.0 * xi3;
	N(2, 3) = -(1.0 - xi) * L * eta;
	N(2, 4) = (-xi + 2.0 * xi2 - xi3) * L;
	N(2, 5) = 0.0;

	// Node 2
	N(0, 6 + 0) = xi;
	N(0, 6 + 1) = 6.0 * (-xi + xi2) * eta;
	N(0, 6 + 2) = 6.0 * (-xi + xi2) * zeta;
	N(0, 6 + 3) = 0.0;
	N(0, 6 + 4) = (-2.0 * xi + 3.0 * xi2) * L * zeta;
	N(0, 6 + 5) = (2.0 * xi - 3.0 * xi2) * L * eta;

	N(1, 6 + 0) = 0.0;
	N(1, 6 + 1) = 3.0 * xi2 - 2.0 * xi3;
	N(1, 6 + 2) = 0.0;
	N(1, 6 + 3) = -L * xi * zeta;
	N(1, 6 + 4) = 0.0;
	N(1, 6 + 5) = (-xi2 + xi3) * L;

	N(2, 6 + 0) = 0.0;
	N(2, 6 + 1) = 0.0;
	N(2, 6 + 2) = 3.0 * xi2 - 2.0 * xi3;
	N(2, 6 + 3) = -L * xi * eta;
	N(2, 6 + 4) = (xi2 - xi3) * L;
	N(2, 6 + 5) = 0.0;
}

class Fem1DElementBeamTests : public ::testing::Test
{
public:
	static const int m_numberNodes = 5;

	std::array<size_t, 2> m_nodeIds;
	SurgSim::Math::OdeState m_restState;
	double m_expectedVolume;
	double m_rho, m_E, m_nu, m_L;
	double m_radius;
	Quaterniond m_orientation;
	Vector forceVector;
	SparseMatrix massMatrix;
	SparseMatrix dampingMatrix;
	SparseMatrix stiffnessMatrix;

	void SetUp() override
	{
		using SurgSim::Math::getSubVector;

		m_rho = 1000.0;
		m_E = 1e6;
		m_nu = 0.45;
		m_radius = 0.01;
		m_L = 1.0;
		m_expectedVolume = m_L * (M_PI * m_radius * m_radius);

		// Beam is made of node 3 and 1 in a bigger system containing m_numberNodes nodes (at least 4)
		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;

		m_restState.setNumDof(6, m_numberNodes);

		m_orientation.coeffs().setRandom();
		m_orientation.normalize();

		Vector3d firstExtremity(0.1, 1.2, 2.3);
		Vector3d secondExtremity = firstExtremity + m_orientation._transformVector(Vector3d(m_L, 0.0, 0.0));

		Vector& x = m_restState.getPositions();
		getSubVector(x, m_nodeIds[0], 6).segment<3>(0) = firstExtremity;
		getSubVector(x, m_nodeIds[1], 6).segment<3>(0) = secondExtremity;

		// Initialize the global matrices for calculations.
		std::vector<size_t> nodeIdsVectorForm(m_nodeIds.begin(), m_nodeIds.end());
		forceVector = Vector::Zero(6 * m_numberNodes);
		Fem1DElementBeam beam(m_nodeIds);
		massMatrix.resize(6 * m_numberNodes, 6 * m_numberNodes);
		beam.assembleMatrixBlocks(SurgSim::Math::Matrix::Zero(12, 12),
								  nodeIdsVectorForm, 6, &massMatrix, true);
		massMatrix.makeCompressed();

		dampingMatrix.resize(6 * m_numberNodes, 6 * m_numberNodes);
		beam.assembleMatrixBlocks(SurgSim::Math::Matrix::Zero(12, 12),
								  nodeIdsVectorForm, 6, &dampingMatrix, true);
		dampingMatrix.makeCompressed();

		stiffnessMatrix.resize(6 * m_numberNodes, 6 * m_numberNodes);
		beam.assembleMatrixBlocks(SurgSim::Math::Matrix::Zero(12, 12),
								  nodeIdsVectorForm, 6, &stiffnessMatrix, true);
		stiffnessMatrix.makeCompressed();
	}

	void getExpectedMassMatrix(Eigen::Ref<SurgSim::Math::Matrix> mass)
	{
		double& L = m_L;
		double L2 = L * L;
		double A = M_PI * m_radius * m_radius;

		double Iz = M_PI_4 * m_radius * m_radius * m_radius * m_radius;
		double Iy = M_PI_4 * m_radius * m_radius * m_radius * m_radius;
		double I = Iz + Iy;

		Eigen::Matrix<double, 12, 12> untransformedMass;

		untransformedMass.col(0) <<
								 1.0 / 3.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									 1.0 / 6.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		untransformedMass.col(1) <<
								 0.0, 13.0 / 35.0 + 6.0 * Iz / (5.0 * A * L2), 0.0, 0.0, 0.0,  11.0 * L / 210.0 + Iz / (10.0 * A * L),
									  0.0,  9.0 / 70.0 - 6.0 * Iz / (5.0 * A * L2), 0.0, 0.0, 0.0, -13.0 * L / 420.0 + Iz / (10.0 * A * L);

		untransformedMass.col(2) <<
								 0.0, 0.0, 13.0 / 35.0 + 6.0 * Iy / (5.0 * A * L2), 0.0, -11.0 * L / 210.0 - Iy / (10.0 * A * L), 0.0,
									  0.0, 0.0,  9.0 / 70.0 - 6.0 * Iy / (5.0 * A * L2), 0.0,  13.0 * L / 420.0 - Iy / (10.0 * A * L), 0.0;

		untransformedMass.col(3) <<
								 0.0, 0.0, 0.0, I / (3.0 * A), 0.0, 0.0,
									  0.0, 0.0, 0.0, I / (6.0 * A), 0.0, 0.0;

		untransformedMass.col(4) <<
								 0.0, 0.0, -11.0 * L / 210.0 - Iy / (10.0 * A * L), 0.0,  L2 / 105.0 + 2.0 * Iy / (15.0 * A), 0.0,
									  0.0, 0.0, -13.0 * L / 420.0 + Iy / (10.0 * A * L), 0.0, -L2 / 140.0 - Iy / (30.0 * A), 0.0;

		untransformedMass.col(5) <<
								 0.0, 11.0 * L / 210.0 + Iz / (10.0 * A * L), 0.0, 0.0, 0.0,  L2 / 105.0 + 2.0 * Iz / (15.0 * A),
									  0.0, 13.0 * L / 420.0 - Iz / (10.0 * A * L), 0.0, 0.0, 0.0, -L2 / 140.0 - Iz / (30.0 * A);

		untransformedMass.col(6) <<
								 1.0 / 6.0, 0.0, 0.0, 0.0, 0.0, 0.0,
									 1.0 / 3.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		untransformedMass.col(7) <<
								 0.0,  9.0 / 70.0 - 6.0 * Iz / (5.0 * A * L2), 0.0, 0.0, 0.0,  13.0 * L / 420.0 - Iz / (10.0 * A * L),
									   0.0, 13.0 / 35.0 + 6.0 * Iz / (5.0 * A * L2), 0.0, 0.0, 0.0, -11.0 * L / 210.0 - Iz / (10.0 * A * L);

		untransformedMass.col(8) <<
								 0.0, 0.0,  9.0 / 70.0 - 6.0 * Iy / (5.0 * A * L2), 0.0, -13.0 * L / 420.0 + Iy / (10.0 * A * L), 0.0,
									  0.0, 0.0, 13.0 / 35.0 + 6.0 * Iy / (5.0 * A * L2), 0.0,  11.0 * L / 210.0 + Iy / (10.0 * A * L), 0.0;

		untransformedMass.col(9) <<
								 0.0, 0.0, 0.0, I / (6.0 * A), 0.0, 0.0,
									  0.0, 0.0, 0.0, I / (3.0 * A), 0.0, 0.0;

		untransformedMass.col(10) <<
								  0.0, 0.0, 13.0 * L / 420.0 - Iy / (10.0 * A * L), 0.0, -L2 / 140.0 - Iy / (30.0 * A), 0.0,
									   0.0, 0.0, 11.0 * L / 210.0 + Iy / (10.0 * A * L), 0.0,  L2 / 105.0 + 2 * Iy / (15.0 * A), 0.0;

		untransformedMass.col(11) <<
								  0.0, -13.0 * L / 420.0 + Iz / (10.0 * A * L), 0.0, 0.0, 0.0, -L2 / 140.0 - Iz / (30.0 * A),
									   0.0, -11.0 * L / 210.0 - Iz / (10.0 * A * L), 0.0, 0.0, 0.0,  L2 / 105.0 + 2 * Iz / (15.0 * A);

		untransformedMass *= m_rho * m_expectedVolume;

		mass.setZero();
		placeIntoAssembly(untransformedMass, mass);
	}

	void getExpectedMassMatrix2(Eigen::Ref<SurgSim::Math::Matrix> mass)
	{
		// Kinetic Energy = K = \int_V 1/2 rho v^2 dV, with rho the density and v the velocity of a volume particle
		// The shape function N transforms local coordinates into global coordinates, x = N q,
		//      N = (N_1^x     0     0 N_2^x     0     0)
		//          (    0 N_1^y     0     0 N_2^y     0)
		//          (    0     0 N_1^z     0     0 N_2^z)
		//
		// Using Lagrange's equation, we find
		// d\dt (dK/d\dot{q}_i) = d\dt( 1/2.rho \int_V d/d\dot(q}_i ( \dot{q}^T.N^T.N.\dot{q} ) dV )
		//                      [Note d/dx(ax \cdot ax) = 2 (a \cdot ax)]
		//                      = rho d\dt( \int_V N^T.N.\dot{q} dV )
		//                      = rho d\dt( \int_V N^T.N dV \dot{q})
		//                      = rho \int_V N^T.N dV \ddot{q}
		//
		// Therefore the mass 'M' is
		//    M = rho \int_V N^T(x,y,z).N(x,y,z) dV
		//      = rho \int_V f(x,y,z) dV
		//
		// Integrate over the cylindrical volume of the beam
		//    M = rho \int_0^L dx \int_{-R}^R dy \int_{-\sqrt{R^2-y^2}}^{\sqrt{R^2-y^2}} dz [f(x,y,z)]
		//
		// Let
		//    x = l \xi
		//    y = l \eta
		//    z = l \zeta
		// Then
		//    \int_0^L dx
		//      => \int_0^1 l d\xi
		//    \int_{-R}^R dy
		//      => \int_{-R/l}^{R/l} l d\eta
		//    \int_{-\sqrt{R^2-y^2}}^{\sqrt{R^2-y^2}} dz
		//      => \int_{-\sqrt{(R/l)^2-\eta^2}}^{\sqrt{(R/l)^2-\eta^2}} l d\zeta
		//
		// We know 'f' in transformed coordinates, so we can directly calculate
		//    M = \rho l^3 ...
		//           \int_0^1 d\xi ...
		//           \int_{-R/l}^{R/l} d\eta ...
		//           \int_{-\sqrt{(R/l)^2-\eta^2}}^{\sqrt{(R/l)^2-\eta^2}} d\zeta ...
		//           [f(\xi, \eta, \zeta)]

		using std::pow;
		using std::sqrt;

		Eigen::Matrix<double, 3, 12> N;
		Eigen::Matrix<double, 12, 12> untransformedMass = Eigen::Matrix<double, 12, 12>::Zero();

		for (int i = 0; i < 4; i++)
		{
			double xi = 0.5 * SurgSim::Math::gaussQuadrature4Points[i].point + 0.5;
			double hXi = 0.5 * SurgSim::Math::gaussQuadrature4Points[i].weight;

			for (int j = 0; j < 100; j++)
			{
				double eta = m_radius / m_L * SurgSim::Math::gaussQuadrature100Points[j].point;
				double hEta = m_radius / m_L * SurgSim::Math::gaussQuadrature100Points[j].weight;

				for (int k = 0; k < 2; k++)
				{
					double zeta = sqrt(pow(m_radius / m_L, 2.0) - pow(eta, 2.0))
								  * SurgSim::Math::gaussQuadrature2Points[k].point;
					double hZeta = sqrt(pow(m_radius / m_L, 2.0) - pow(eta, 2.0))
								   * SurgSim::Math::gaussQuadrature2Points[k].weight;

					computeShapeFunction(xi, eta, zeta, m_L, N);

					untransformedMass += N.transpose() * N * hZeta * hEta * hXi;
				}
			}
		}

		untransformedMass *= m_rho * pow(m_L, 3.0);

		mass.setZero();
		placeIntoAssembly(untransformedMass, mass);
	}

	void getExpectedStiffnessMatrix(Eigen::Ref<SurgSim::Math::Matrix> stiffness)
	{
		double& L = m_L;
		double L2 = L * L;
		double L3 = L2 * L;
		double A = (M_PI * m_radius * m_radius);

		double Iz = M_PI / 4.0 * (m_radius * m_radius * m_radius * m_radius);
		double Iy = M_PI / 4.0 * (m_radius * m_radius * m_radius * m_radius);
		double I = Iz + Iy;

		double G = m_E / (2.0 * (1.0 + m_nu));

		double asy = A * 5. / 6.;
		double asz = asy;

		double phi_y = 12.0 * m_E * Iz / (G * asy * L2);
		double phi_z = 12.0 * m_E * Iy / (G * asz * L2);

		Eigen::Matrix<double, 12, 12> untransformedStiffness;

		untransformedStiffness.col(0) <<
									  m_E* A / L, 0.0, 0.0, 0.0, 0.0, 0.0,
										   -m_E* A / L, 0.0, 0.0, 0.0, 0.0, 0.0;

		untransformedStiffness.col(1) <<
									  0.0,  12.0 * m_E* Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, 6.0 * m_E* Iz / L2 / (1 + phi_y),
											0.0, -12.0 * m_E* Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, 6.0 * m_E* Iz / L2 / (1 + phi_y);

		untransformedStiffness.col(2) <<
									  0.0, 0.0,  12.0 * m_E* Iy / L3 / (1 + phi_z), 0.0, -6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0,
										   0.0, 0.0, -12.0 * m_E* Iy / L3 / (1 + phi_z), 0.0, -6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0;

		untransformedStiffness.col(3) <<
									  0.0, 0.0, 0.0, G* I / L, 0.0, 0.0,
										   0.0, 0.0, 0.0, -G* I / L, 0.0, 0.0;

		untransformedStiffness.col(4) <<
									  0.0, 0.0, -6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0, (4.0 + phi_z) * m_E* Iy / L / (1 + phi_z), 0.0,
										   0.0, 0.0,  6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0, (2.0 - phi_z) * m_E* Iy / L / (1 + phi_z), 0.0;

		untransformedStiffness.col(5) <<
									  0.0,  6.0 * m_E* Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (4.0 + phi_y) * m_E* Iz / L / (1 + phi_y),
											0.0, -6.0 * m_E* Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (2.0 - phi_y) * m_E* Iz / L / (1 + phi_y);

		untransformedStiffness.col(6) <<
									  -m_E* A / L, 0.0, 0.0, 0.0, 0.0, 0.0,
									  m_E* A / L, 0.0, 0.0, 0.0, 0.0, 0.0;

		untransformedStiffness.col(7) <<
									  0.0, -12.0 * m_E* Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, -6.0 * m_E* Iz / L2 / (1 + phi_y),
										   0.0,  12.0 * m_E* Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, -6.0 * m_E* Iz / L2 / (1 + phi_y);

		untransformedStiffness.col(8) <<
									  0.0, 0.0, -12.0 * m_E* Iy / L3 / (1 + phi_z), 0.0, 6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0,
										   0.0, 0.0,  12.0 * m_E* Iy / L3 / (1 + phi_z), 0.0, 6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0;

		untransformedStiffness.col(9) <<
									  0.0, 0.0, 0.0, -G* I / L, 0.0, 0.0,
										   0.0, 0.0, 0.0,  G* I / L, 0.0, 0.0;

		untransformedStiffness.col(10) <<
									   0.0, 0.0, -6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0, (2.0 - phi_z) * m_E* Iy / L / (1 + phi_z), 0.0,
											0.0, 0.0,  6.0 * m_E* Iy / L2 / (1 + phi_z), 0.0, (4.0 + phi_z) * m_E* Iy / L / (1 + phi_z), 0.0;

		untransformedStiffness.col(11) <<
									   0.0,  6.0 * m_E* Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (2.0 - phi_y) * m_E* Iz / L / (1 + phi_y),
											 0.0, -6.0 * m_E* Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (4.0 + phi_y) * m_E* Iz / L / (1 + phi_y);

		stiffness.setZero();
		placeIntoAssembly(untransformedStiffness, stiffness);
	}

	void placeIntoAssembly(const Eigen::Ref<SurgSim::Math::Matrix>& in, Eigen::Ref<SurgSim::Math::Matrix> out)
	{
		std::vector<size_t> nodeIdsVectorForm(m_nodeIds.begin(), m_nodeIds.end());

		// Transform into correct coordinates and correct place in matrix
		std::shared_ptr<MockFem1DElement> beam = getBeam();
		const Eigen::Matrix<double, 12, 12>& r = beam->getInitialRotation();

		SurgSim::Math::addSubMatrix(r * in * r.transpose(), nodeIdsVectorForm, 6, &out);
	}

	std::shared_ptr<MockFem1DElement> getBeam()
	{
		auto beam = std::make_shared<MockFem1DElement>(m_nodeIds);
		beam->setRadius(m_radius);
		beam->setMassDensity(m_rho);
		beam->setPoissonRatio(m_nu);
		beam->setYoungModulus(m_E);
		beam->initialize(m_restState);
		return beam;
	}
};

TEST_F(Fem1DElementBeamTests, ConstructorTest)
{
	ASSERT_NO_THROW(
	{ MockFem1DElement beam(m_nodeIds); });
}

TEST_F(Fem1DElementBeamTests, NodeIdsTest)
{
	Fem1DElementBeam beam(m_nodeIds);
	EXPECT_EQ(2u, beam.getNumNodes());
	EXPECT_EQ(2u, beam.getNodeIds().size());
	for (int i = 0; i < 2; i++)
	{
		EXPECT_EQ(m_nodeIds[i], beam.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], beam.getNodeIds()[i]);
	}
}

TEST_F(Fem1DElementBeamTests, setGetRadiusTest)
{
	Fem1DElementBeam beam(m_nodeIds);

	// Default radius = 0
	EXPECT_DOUBLE_EQ(0.0, beam.getRadius());
	// Set to a valid radius
	beam.setRadius(1.54);
	EXPECT_DOUBLE_EQ(1.54, beam.getRadius());
	// Set to an invalid radius
	EXPECT_ANY_THROW(beam.setRadius(0.0));
	EXPECT_ANY_THROW(beam.setRadius(-9.4));
}

TEST_F(Fem1DElementBeamTests, MaterialParameterTest)
{
	Fem1DElementBeam beam(m_nodeIds);
	beam.setRadius(m_radius);

	// Test the various mode of failure related to the physical parameters
	// This has been already tested in FemElementTests, but this is to make sure this method is called properly
	// So the same behavior should be expected
	{
		// Mass density not set
		ASSERT_ANY_THROW(beam.initialize(m_restState));

		// Poisson Ratio not set
		beam.setMassDensity(-1234.56);
		ASSERT_ANY_THROW(beam.initialize(m_restState));

		// Young modulus not set
		beam.setPoissonRatio(0.55);
		ASSERT_ANY_THROW(beam.initialize(m_restState));

		// Invalid mass density
		beam.setYoungModulus(-4321.33);
		ASSERT_ANY_THROW(beam.initialize(m_restState));

		// Invalid Poisson ratio
		beam.setMassDensity(m_rho);
		ASSERT_ANY_THROW(beam.initialize(m_restState));

		// Invalid Young modulus
		beam.setPoissonRatio(m_nu);
		ASSERT_ANY_THROW(beam.initialize(m_restState));

		beam.setYoungModulus(m_E);
		ASSERT_NO_THROW(beam.initialize(m_restState));
	}
}

TEST_F(Fem1DElementBeamTests, VolumeTest)
{
	std::shared_ptr<MockFem1DElement> beam = getBeam();
	EXPECT_NEAR(beam->getVolume(m_restState), m_expectedVolume, 1e-10);
}

TEST_F(Fem1DElementBeamTests, RestLengthTest)
{
	std::shared_ptr<MockFem1DElement> beam = getBeam();
	EXPECT_NEAR(beam->getRestLength(), m_L, 1e-10);
}

TEST_F(Fem1DElementBeamTests, InitialRotationTest)
{
	std::shared_ptr<MockFem1DElement> beam = getBeam();

	// Use a mask to test the structure of the rotation matrix R0 (4 digonal block 3x3 matrix and 0 elsewhere)
	Eigen::Matrix<double, 12, 12> mask;
	mask.setOnes();
	mask.block<3, 3>(0, 0).setZero();
	mask.block<3, 3>(3, 3).setZero();
	mask.block<3, 3>(6, 6).setZero();
	mask.block<3, 3>(9, 9).setZero();
	EXPECT_TRUE(beam->getInitialRotation().cwiseProduct(mask).isZero());

	// Only the 1st direction of the frame can be compared as the 2 other ones are randomly
	// chosen (can be any 2 vectors forming an Orthonormal frame)
	Vector3d expected_i = m_orientation.matrix().col(0);
	Vector3d i_0 = beam->getInitialRotation().block<3, 3>(0, 0).col(0);
	Vector3d i_1 = beam->getInitialRotation().block<3, 3>(3, 3).col(0);
	Vector3d i_2 = beam->getInitialRotation().block<3, 3>(6, 6).col(0);
	Vector3d i_3 = beam->getInitialRotation().block<3, 3>(9, 9).col(0);
	EXPECT_TRUE(i_0.isApprox(expected_i));
	EXPECT_TRUE(i_1.isApprox(expected_i));
	EXPECT_TRUE(i_2.isApprox(expected_i));
	EXPECT_TRUE(i_3.isApprox(expected_i));
}

TEST_F(Fem1DElementBeamTests, CoordinateTests)
{
	Fem1DElementBeam element(m_nodeIds);

	Vector validNaturalCoordinate(2);
	Vector validNaturalCoordinate2(2);
	Vector invalidNaturalCoordinateSumNot1(2);
	Vector invalidNaturalCoordinateNegativeValue(2);
	Vector invalidNaturalCoordinateSize1(1), invalidNaturalCoordinateSize3(3);
	Vector3d expectedA(0.1, 1.2, 2.3);
	Vector3d expectedB = expectedA + m_orientation._transformVector(Vector3d(m_L, 0.0, 0.0));

	validNaturalCoordinate << 0.4, 0.6;
	validNaturalCoordinate2 << -1e-11, 1 + 1e-11;
	invalidNaturalCoordinateSumNot1 << 0.5, 0.6;
	invalidNaturalCoordinateNegativeValue << 1.4, -0.4;
	invalidNaturalCoordinateSize1 << 1.0;
	invalidNaturalCoordinateSize3 << 0.2, 0.2, 0.6;
	EXPECT_TRUE(element.isValidCoordinate(validNaturalCoordinate));
	EXPECT_TRUE(element.isValidCoordinate(validNaturalCoordinate2));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSumNot1));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateNegativeValue));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSize1));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSize3));

	Vector naturalCoordinateA(2), naturalCoordinateB(2), naturalCoordinateMiddle(2);
	Vector ptA, ptB, ptMiddle;
	naturalCoordinateA << 1.0, 0.0;
	naturalCoordinateB << 0.0, 1.0;
	naturalCoordinateMiddle << 1.0 / 2.0, 1.0 / 2.0;
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateNegativeValue), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSize1), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSize3), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSumNot1), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(ptA = element.computeCartesianCoordinate(m_restState, naturalCoordinateA));
	EXPECT_NO_THROW(ptB = element.computeCartesianCoordinate(m_restState, naturalCoordinateB));
	EXPECT_NO_THROW(ptMiddle = element.computeCartesianCoordinate(m_restState, naturalCoordinateMiddle));
	EXPECT_TRUE(ptA.isApprox(expectedA));
	EXPECT_TRUE(ptB.isApprox(expectedB));
	EXPECT_TRUE(ptMiddle.isApprox((expectedA + expectedB) * 0.5));

	Vector3d cartesian(0.1, 1.2, 2.3);
	EXPECT_THROW(element.computeNaturalCoordinate(m_restState, cartesian), SurgSim::Framework::AssertionFailure);
}

TEST_F(Fem1DElementBeamTests, ForceAndMatricesTest)
{
	using SurgSim::Math::Matrix;
	using SurgSim::Math::Vector;
	using SurgSim::Math::getSubVector;

	std::shared_ptr<MockFem1DElement> beam = getBeam();

	Vector vectorOnes = Vector::Ones(6 * m_numberNodes);

	forceVector.setZero();
	SurgSim::Math::clearMatrix(&massMatrix);
	SurgSim::Math::clearMatrix(&dampingMatrix);
	SurgSim::Math::clearMatrix(&stiffnessMatrix);

	Matrix expectedMass(6 * m_numberNodes, 6 * m_numberNodes);
	Matrix expectedMass2(6 * m_numberNodes, 6 * m_numberNodes);
	Matrix expectedDamping = Matrix(dampingMatrix);
	Matrix expectedStiffness(6 * m_numberNodes, 6 * m_numberNodes);

	getExpectedMassMatrix(expectedMass);
	getExpectedMassMatrix2(expectedMass2);
	getExpectedStiffnessMatrix(expectedStiffness);

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	beam->addForce(m_restState, &forceVector);
	EXPECT_TRUE(forceVector.isZero());

	beam->addMass(m_restState, &massMatrix);
	EXPECT_TRUE(massMatrix.isApprox(expectedMass)) << "Expected Mass:" << std::endl << expectedMass << std::endl <<
			"Mass Matrix:" << std::endl << massMatrix << std::endl;
	EXPECT_TRUE(massMatrix.isApprox(expectedMass2, 1e-6)) << "Expected Mass 2:" << std::endl << expectedMass <<
			std::endl << "Mass Matrix:" << std::endl << massMatrix << std::endl;

	beam->addDamping(m_restState, &dampingMatrix);
	EXPECT_TRUE(dampingMatrix.isApprox(expectedDamping));

	beam->addStiffness(m_restState, &stiffnessMatrix);
	EXPECT_TRUE(stiffnessMatrix.isApprox(expectedStiffness));

	forceVector.setZero();
	SurgSim::Math::clearMatrix(&massMatrix);
	SurgSim::Math::clearMatrix(&dampingMatrix);
	SurgSim::Math::clearMatrix(&stiffnessMatrix);

	beam->addFMDK(m_restState, &forceVector, &massMatrix, &dampingMatrix, &stiffnessMatrix);
	EXPECT_TRUE(forceVector.isZero());
	EXPECT_TRUE(massMatrix.isApprox(expectedMass));
	EXPECT_TRUE(massMatrix.isApprox(expectedMass2, 1e-6));
	EXPECT_TRUE(dampingMatrix.isApprox(expectedDamping));
	EXPECT_TRUE(stiffnessMatrix.isApprox(expectedStiffness));

	// Test addMatVec API with Mass component only
	forceVector.setZero();
	beam->addMatVec(m_restState, 1.0, 0.0, 0.0, vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 6 * m_numberNodes; rowId++)
	{
		EXPECT_NEAR(expectedMass.row(rowId).sum(), forceVector[rowId], epsilon);
		EXPECT_NEAR(expectedMass2.row(rowId).sum(), forceVector[rowId], 1e-6);
	}
	// Test addMatVec API with Damping component only
	forceVector.setZero();
	beam->addMatVec(m_restState, 0.0, 1.0, 0.0, vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 6 * m_numberNodes; rowId++)
	{
		EXPECT_NEAR(expectedDamping.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Stiffness component only
	forceVector.setZero();
	beam->addMatVec(m_restState, 0.0, 0.0, 1.0, vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 6 * m_numberNodes; rowId++)
	{
		EXPECT_NEAR(expectedStiffness.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with mix Mass/Damping/Stiffness components
	forceVector.setZero();
	beam->addMatVec(m_restState, 1.0, 2.0, 3.0, vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 6 * m_numberNodes; rowId++)
	{
		double expectedCoef = 1.0 * expectedMass.row(rowId).sum()
							  + 2.0 * expectedDamping.row(rowId).sum()
							  + 3.0 * expectedStiffness.row(rowId).sum();
		EXPECT_NEAR(expectedCoef, forceVector[rowId], epsilon);

		expectedCoef = 1.0 * expectedMass2.row(rowId).sum()
					   + 2.0 * expectedDamping.row(rowId).sum()
					   + 3.0 * expectedStiffness.row(rowId).sum();
		EXPECT_NEAR(expectedCoef, forceVector[rowId], 1e-6);
	}
}
