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

#include "SurgSim/Math/GaussLegendreQuadrature.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/FemElement1DBeam.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::FemElement1DBeam;

namespace
{
const double epsilon = 1e-9;
};

class MockFemElement1D : public FemElement1DBeam
{
public:
	MockFemElement1D(std::array<unsigned int, 2> nodeIds, const DeformableRepresentationState& restState)
		: FemElement1DBeam(nodeIds, restState)
	{
	}

	const Eigen::Matrix<double, 12, 12, Eigen::DontAlign>& getInitialRotation() const
	{
		return m_R0;
	}

	double getRestLength() const
	{
		return m_restLength;
	}
};

class FemElement1DBeamTests : public ::testing::Test
{
public:
	static const int m_numberNodes = 5;

	std::array<unsigned int, 2> m_nodeIds;
	DeformableRepresentationState m_restState;
	double m_expectedVolume;
	double m_rho, m_E, m_nu, m_L;
	double m_radius;
	Quaterniond m_orientation;

	virtual void SetUp() override
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

		Eigen::Matrix<double, 12, 12, Eigen::DontAlign> untransformedStiffness;

		untransformedStiffness.col(0) <<
			 m_E * A / L, 0.0, 0.0, 0.0, 0.0, 0.0,
			-m_E * A / L, 0.0, 0.0, 0.0, 0.0, 0.0;

		untransformedStiffness.col(1) <<
			0.0,  12.0 * m_E * Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, 6.0 * m_E * Iz / L2 / (1 + phi_y),
			0.0, -12.0 * m_E * Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, 6.0 * m_E * Iz / L2 / (1 + phi_y);

		untransformedStiffness.col(2) <<
			0.0, 0.0,  12.0 * m_E * Iy / L3 / (1 + phi_z), 0.0, -6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0,
			0.0, 0.0, -12.0 * m_E * Iy / L3 / (1 + phi_z), 0.0, -6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0;

		untransformedStiffness.col(3) <<
			0.0, 0.0, 0.0, G * I / L, 0.0, 0.0,
			0.0, 0.0, 0.0, -G * I / L, 0.0, 0.0;

		untransformedStiffness.col(4) <<
			0.0, 0.0, -6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0, (4.0 + phi_z) * m_E * Iy / L / (1 + phi_z), 0.0,
			0.0, 0.0,  6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0, (2.0 - phi_z) * m_E * Iy / L / (1 + phi_z), 0.0;

		untransformedStiffness.col(5) <<
			0.0,  6.0 * m_E * Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (4.0 + phi_y) * m_E * Iz / L / (1 + phi_y),
			0.0, -6.0 * m_E * Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (2.0 - phi_y) * m_E * Iz / L / (1 + phi_y);

		untransformedStiffness.col(6) <<
			-m_E * A / L, 0.0, 0.0, 0.0, 0.0, 0.0,
			 m_E * A / L, 0.0, 0.0, 0.0, 0.0, 0.0;

		untransformedStiffness.col(7) <<
			0.0, -12.0 * m_E * Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, -6.0 * m_E * Iz / L2 / (1 + phi_y),
			0.0,  12.0 * m_E * Iz / L3 / (1 + phi_y), 0.0, 0.0, 0.0, -6.0 * m_E * Iz / L2 / (1 + phi_y);

		untransformedStiffness.col(8) <<
			0.0, 0.0, -12.0 * m_E * Iy / L3 / (1 + phi_z), 0.0, 6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0,
			0.0, 0.0,  12.0 * m_E * Iy / L3 / (1 + phi_z), 0.0, 6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0;

		untransformedStiffness.col(9) <<
			0.0, 0.0, 0.0, -G * I / L, 0.0, 0.0,
			0.0, 0.0, 0.0,  G * I / L, 0.0, 0.0;

		untransformedStiffness.col(10) <<
			0.0, 0.0, -6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0, (2.0 - phi_z) * m_E * Iy / L / (1 + phi_z), 0.0,
			0.0, 0.0,  6.0 * m_E * Iy / L2 / (1 + phi_z), 0.0, (4.0 + phi_z) * m_E * Iy / L / (1 + phi_z), 0.0;

		untransformedStiffness.col(11) <<
			0.0,  6.0 * m_E * Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (2.0 - phi_y) * m_E * Iz / L / (1 + phi_y),
			0.0, -6.0 * m_E * Iz / L2 / (1 + phi_y), 0.0, 0.0, 0.0, (4.0 + phi_y) * m_E * Iz / L / (1 + phi_y);

		stiffness.setZero();
		placeIntoAssembly(untransformedStiffness, stiffness);
	}

	void placeIntoAssembly(const Eigen::Ref<SurgSim::Math::Matrix>& in, Eigen::Ref<SurgSim::Math::Matrix> out)
	{
		std::vector<unsigned int> nodeIdsVectorForm(m_nodeIds.begin(), m_nodeIds.end());

		// Transform into correct coordinates and correct place in matrix
		std::shared_ptr<MockFemElement1D> beam = getBeam();
		const Eigen::Matrix<double, 12, 12, Eigen::DontAlign>& r = beam->getInitialRotation();

		SurgSim::Math::addSubMatrix(r.transpose() * in * r, nodeIdsVectorForm, 6, &out);
	}

	std::shared_ptr<MockFemElement1D> getBeam()
	{
		auto beam = std::make_shared<MockFemElement1D>(m_nodeIds, m_restState);
		beam->setCrossSectionCircular(m_radius);
		beam->setMassDensity(m_rho);
		beam->setPoissonRatio(m_nu);
		beam->setYoungModulus(m_E);
		beam->initialize(m_restState);
		return beam;
	}
};

TEST_F(FemElement1DBeamTests, ConstructorTest)
{
	ASSERT_NO_THROW(
		{ MockFemElement1D beam(m_nodeIds, m_restState); });
	ASSERT_NO_THROW(
		{
			MockFemElement1D* beam = new MockFemElement1D(m_nodeIds, m_restState);
			delete beam;
		});
	ASSERT_NO_THROW(
		{ std::shared_ptr<MockFemElement1D> beam = std::make_shared<MockFemElement1D>(m_nodeIds, m_restState); });
}

TEST_F(FemElement1DBeamTests, NodeIdsTest)
{
	FemElement1DBeam beam(m_nodeIds, m_restState);
	EXPECT_EQ(2u, beam.getNumNodes());
	EXPECT_EQ(2u, beam.getNodeIds().size());
	for (int i = 0; i < 2; i++)
	{
		EXPECT_EQ(m_nodeIds[i], beam.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], beam.getNodeIds()[i]);
	}
}

TEST_F(FemElement1DBeamTests, setGetRadiusTest)
{
	FemElement1DBeam beam(m_nodeIds, m_restState);

	// Default radius = 0
	EXPECT_DOUBLE_EQ(0.0, beam.getCrossSectionCircular());
	// Set to a valid radius
	beam.setCrossSectionCircular(1.54);
	EXPECT_DOUBLE_EQ(1.54, beam.getCrossSectionCircular());
	// Set to an invalid radius
	EXPECT_ANY_THROW(beam.setCrossSectionCircular(0.0));
	EXPECT_ANY_THROW(beam.setCrossSectionCircular(-9.4));
}

TEST_F(FemElement1DBeamTests, MaterialParameterTest)
{
	FemElement1DBeam beam(m_nodeIds, m_restState);
	beam.setCrossSectionCircular(m_radius);

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

TEST_F(FemElement1DBeamTests, VolumeTest)
{
	std::shared_ptr<MockFemElement1D> beam = getBeam();
	EXPECT_NEAR(beam->getVolume(m_restState), m_expectedVolume, 1e-10);
}

TEST_F(FemElement1DBeamTests, RestLengthTest)
{
	std::shared_ptr<MockFemElement1D> beam = getBeam();
	EXPECT_NEAR(beam->getRestLength(), m_L, 1e-10);
}

TEST_F(FemElement1DBeamTests, InitialRotationTest)
{
	std::shared_ptr<MockFemElement1D> beam = getBeam();

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

TEST_F(FemElement1DBeamTests, ForceAndMatricesTest)
{
	using SurgSim::Math::Matrix;
	using SurgSim::Math::Vector;
	using SurgSim::Math::getSubVector;

	std::shared_ptr<MockFemElement1D> beam = getBeam();

	Vector vectorOnes = Vector::Ones(6 * m_numberNodes);

	Vector forceVector = Vector::Zero(6 * m_numberNodes);
	Matrix massMatrix = Matrix::Zero(6 * m_numberNodes, 6 * m_numberNodes);
	Matrix dampingMatrix = Matrix::Zero(6 * m_numberNodes, 6 * m_numberNodes);
	Matrix stiffnessMatrix = Matrix::Zero(6 * m_numberNodes, 6 * m_numberNodes);

	Matrix expectedMass(6 * m_numberNodes, 6 * m_numberNodes);
	Matrix expectedDamping = Matrix(dampingMatrix);
	Matrix expectedStiffness(6 * m_numberNodes, 6 * m_numberNodes);

	getExpectedMassMatrix(expectedMass);
	getExpectedStiffnessMatrix(expectedStiffness);

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	beam->addForce(m_restState, &forceVector);
	EXPECT_TRUE(forceVector.isZero());

	beam->addMass(m_restState, &massMatrix);
	EXPECT_TRUE(massMatrix.isApprox(expectedMass));

	beam->addDamping(m_restState, &dampingMatrix);
	EXPECT_TRUE(dampingMatrix.isApprox(expectedDamping));

	beam->addStiffness(m_restState, &stiffnessMatrix);
	EXPECT_TRUE(stiffnessMatrix.isApprox(expectedStiffness));

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	beam->addFMDK(m_restState, &forceVector, &massMatrix, &dampingMatrix, &stiffnessMatrix);
	EXPECT_TRUE(forceVector.isZero());
	EXPECT_TRUE(massMatrix.isApprox(expectedMass));
	EXPECT_TRUE(dampingMatrix.isApprox(expectedDamping));
	EXPECT_TRUE(stiffnessMatrix.isApprox(expectedStiffness));

	// Test addMatVec API with Mass component only
	forceVector.setZero();
	beam->addMatVec(m_restState, 1.0, 0.0, 0.0, vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 6 * m_numberNodes; rowId++)
	{
		EXPECT_NEAR(expectedMass.row(rowId).sum(), forceVector[rowId], epsilon);
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
	}
}
