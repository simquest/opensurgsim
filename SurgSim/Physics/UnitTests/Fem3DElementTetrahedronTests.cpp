// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"

using SurgSim::Physics::Fem3DElementTetrahedron;
using SurgSim::Math::clearMatrix;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;
using SurgSim::Math::SparseMatrix;

namespace
{
/// Useful function to compute a shape function
/// \param i ith shape function
/// \param ai, bi, ci, di The shape functions parameters
/// \param p The point to evaluate the shape function at
/// \return Ni(p) = 1/6V . (ai + bi.px + ci.py + di.pz)
double N(size_t i, double V, double* ai, double* bi, double* ci, double* di, const Vector3d& p)
{
	double inv6V = 1.0 / (6.0 * V);
	return inv6V * (ai[i] + bi[i] * p[0] + ci[i] * p[1] + di[i] * p[2]);
}

const double epsilon = 1e-8;
};

class MockFem3DElementTet : public Fem3DElementTetrahedron
{
public:
	explicit MockFem3DElementTet(std::array<size_t, 4> nodeIds) : Fem3DElementTetrahedron(nodeIds)
	{
	}

	double getRestVolume() const
	{
		return m_restVolume;
	}

	void getShapeFunction(int i, double* ai, double* bi, double* ci, double* di) const
	{
		*ai = m_ai[i];
		*bi = m_bi[i];
		*ci = m_ci[i];
		*di = m_di[i];
	}

	const Eigen::Matrix<double, 12, 1>& getInitialPosition() const
	{
		return m_x0;
	}

	void setupInitialParams(const SurgSim::Math::OdeState& state,
							double massDensity,
							double poissonRatio,
							double youngModulus)
	{
		setMassDensity(massDensity);
		setPoissonRatio(poissonRatio);
		setYoungModulus(youngModulus);
		initialize(state);
	}
};

class Fem3DElementTetrahedronTests : public ::testing::Test
{
public:
	std::array<size_t, 4> m_nodeIds;
	SurgSim::Math::OdeState m_restState;
	double m_expectedVolume;
	Eigen::Matrix<double, 12, 1> m_expectedX0;
	double m_rho, m_E, m_nu;
	SurgSim::Math::Matrix m_expectedMassMatrix, m_expectedDampingMatrix;
	SurgSim::Math::Matrix m_expectedStiffnessMatrix, m_expectedStiffnessMatrix2;
	SurgSim::Math::Vector m_vectorOnes;

	void SetUp() override
	{
		using SurgSim::Math::getSubVector;
		using SurgSim::Math::getSubMatrix;
		using SurgSim::Math::addSubMatrix;

		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;
		m_nodeIds[2] = 14;
		m_nodeIds[3] = 9;
		std::vector<size_t> m_nodeIdsVectorForm; // Useful for assembly helper function
		m_nodeIdsVectorForm.push_back(m_nodeIds[0]);
		m_nodeIdsVectorForm.push_back(m_nodeIds[1]);
		m_nodeIdsVectorForm.push_back(m_nodeIds[2]);
		m_nodeIdsVectorForm.push_back(m_nodeIds[3]);

		m_restState.setNumDof(3, 15);
		Vector& x0 = m_restState.getPositions();
		// Tet is aligned with the axis (X,Y,Z), centered on (0.1, 1.2, 2.3), embedded in a cube of size 1
		getSubVector(m_expectedX0, 0, 3) = getSubVector(x0, m_nodeIds[0], 3) = Vector3d(0.1, 1.2, 2.3);
		getSubVector(m_expectedX0, 1, 3) = getSubVector(x0, m_nodeIds[1], 3) = Vector3d(1.1, 1.2, 2.3);
		getSubVector(m_expectedX0, 2, 3) = getSubVector(x0, m_nodeIds[2], 3) = Vector3d(0.1, 2.2, 2.3);
		getSubVector(m_expectedX0, 3, 3) = getSubVector(x0, m_nodeIds[3], 3) = Vector3d(0.1, 1.2, 3.3);

		// The tet is part of a cube of size 1x1x1 (it occupies 1/6 of the cube's volume)
		m_expectedVolume = 1.0 / 6.0;

		m_rho = 1000.0;
		m_E = 1e6;
		m_nu = 0.45;

		m_expectedMassMatrix.setZero(3 * 15, 3 * 15);
		m_expectedDampingMatrix.setZero(3 * 15, 3 * 15);
		m_expectedStiffnessMatrix.setZero(3 * 15, 3 * 15);
		m_expectedStiffnessMatrix2.setZero(3 * 15, 3 * 15);
		m_vectorOnes.setOnes(3 * 15);

		Eigen::Matrix<double, 12, 12> M = Eigen::Matrix<double, 12, 12>::Zero();
		{
			M.diagonal().setConstant(2.0);
			M.block(0, 3, 9, 9).diagonal().setConstant(1.0);
			M.block(0, 6, 6, 6).diagonal().setConstant(1.0);
			M.block(0, 9, 3, 3).diagonal().setConstant(1.0);
			M.block(3, 0, 9, 9).diagonal().setConstant(1.0);
			M.block(6, 0, 6, 6).diagonal().setConstant(1.0);
			M.block(9, 0, 3, 3).diagonal().setConstant(1.0);
		}
		M *= m_rho * m_expectedVolume / 20.0;
		addSubMatrix(M, m_nodeIdsVectorForm, 3 , &m_expectedMassMatrix);

		Eigen::Matrix<double, 12, 12> K = Eigen::Matrix<double, 12, 12>::Zero();
		{
			// Calculation done by hand from
			// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
			// ai = {}
			// bi = {-1 1 0 0}
			// ci = {-1 0 1 0}
			// di = {-1 0 0 1}
			Eigen::Matrix<double, 6, 12> B = Eigen::Matrix<double, 6, 12>::Zero();
			Eigen::Matrix<double, 6, 6> E = Eigen::Matrix<double, 6, 6>::Zero();

			B(0, 0) = -1;
			B(0, 3) = 1;
			B(1, 1) = -1;
			B(1, 7) = 1;
			B(2, 2) = -1;
			B(2, 11) = 1;
			B(3, 0) = -1;
			B(3, 1) = -1;
			B(3, 4) = 1;
			B(3, 6) = 1;
			B(4, 1) = -1;
			B(4, 2) = -1;
			B(4, 8) = 1;
			B(4, 10) = 1;
			B(5, 0) = -1;
			B(5, 2) = -1;
			B(5, 5) = 1;
			B(5, 9) = 1;
			B *= 1.0 / (6.0 * m_expectedVolume);

			E.block(0, 0, 3, 3).setConstant(m_nu);
			E.block(0, 0, 3, 3).diagonal().setConstant(1.0 - m_nu);
			E.block(3, 3, 3, 3).diagonal().setConstant(0.5 - m_nu);
			E *= m_E / ((1.0 + m_nu) * (1.0 - 2.0 * m_nu));

			K = m_expectedVolume * B.transpose() * E * B;
		}
		addSubMatrix(K, m_nodeIdsVectorForm, 3 , &m_expectedStiffnessMatrix);

		// Expecte stiffness matrix given for our case in:
		// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
		double E = m_E / (12.0 * (1.0 - 2.0 * m_nu) * (1.0 + m_nu));
		double n0 = 1.0 - 2.0 * m_nu;
		double n1 = 1.0 - m_nu;
		K.setZero();

		// Fill up the upper triangle part first (without diagonal elements)
		K(0, 1) = K(0, 2) = K(1, 2) = 1.0;

		K(0, 3) = -2.0 * n1;
		K(0, 4) = -n0;
		K(0, 5) = -n0;
		K(1, 3) = -2.0 * m_nu;
		K(1, 4) = -n0;
		K(2, 3) = -2.0 * m_nu;
		K(2, 5) = -n0;

		K(0, 6) = - n0;
		K(0, 7) = -2.0 * m_nu;
		K(1, 6) = - n0;
		K(1, 7) = -2.0 * n1;
		K(1, 8) = - n0;
		K(2, 7) = - 2.0 * m_nu;
		K(2, 8) = -n0;

		K(0, 9) = - n0;
		K(0, 11) = -2.0 * m_nu;
		K(1, 10) = - n0;
		K(1, 11) = -2.0 * m_nu;
		K(2, 9) = - n0;
		K(2, 10) = - n0;
		K(2, 11) = -2.0 * n1;

		K(3, 7) = K(3, 11) =  2.0 * m_nu;
		K(4, 6) = n0;
		K(5, 9) = n0;
		K(7, 11) = 2.0 * m_nu;
		K(8, 10) = n0;

		K += K.transpose().eval(); // symmetric part (do not forget the .eval() !)

		K.block(0, 0, 3, 3).diagonal().setConstant(4.0 - 6.0 * m_nu); // diagonal elements
		K.block(3, 3, 9, 9).diagonal().setConstant(n0); // diagonal elements
		K(3, 3) = K(7, 7) = K(11, 11) = 2.0 * n1; // diagonal elements

		K *= E;

		addSubMatrix(K, m_nodeIdsVectorForm, 3 , &m_expectedStiffnessMatrix2);
	}
};

extern void testSize(const Vector& v, int expectedSize);
extern void testSize(const Matrix& m, int expectedRows, int expectedCols);

TEST_F(Fem3DElementTetrahedronTests, ConstructorTest)
{
	ASSERT_NO_THROW({MockFem3DElementTet tet(m_nodeIds);});
	ASSERT_NO_THROW({MockFem3DElementTet* tet = new MockFem3DElementTet(m_nodeIds); delete tet;});
	ASSERT_NO_THROW({std::shared_ptr<MockFem3DElementTet> tet =
						 std::make_shared<MockFem3DElementTet>(m_nodeIds);
					});
}

TEST_F(Fem3DElementTetrahedronTests, NodeIdsTest)
{
	Fem3DElementTetrahedron tet(m_nodeIds);
	EXPECT_EQ(4u, tet.getNumNodes());
	EXPECT_EQ(4u, tet.getNodeIds().size());
	for (int i = 0; i < 4; i++)
	{
		EXPECT_EQ(m_nodeIds[i], tet.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], tet.getNodeIds()[i]);
	}
}

TEST_F(Fem3DElementTetrahedronTests, VolumeTest)
{
	MockFem3DElementTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	EXPECT_NEAR(tet.getRestVolume(), m_expectedVolume, 1e-10);
	EXPECT_NEAR(tet.getVolume(m_restState), m_expectedVolume, 1e-10);
}

TEST_F(Fem3DElementTetrahedronTests, CoordinateTests)
{
	Fem3DElementTetrahedron element(m_nodeIds);
	Vector3d expectedA(0.1, 1.2, 2.3);
	Vector3d expectedB(1.1, 1.2, 2.3);
	Vector3d expectedC(0.1, 2.2, 2.3);
	Vector3d expectedD(0.1, 1.2, 3.3);

	Vector validNaturalCoordinate(4);
	Vector validNaturalCoordinate2(4);
	Vector invalidNaturalCoordinateSumNot1(4);
	Vector invalidNaturalCoordinateNegativeValue(4);
	Vector invalidNaturalCoordinateBiggerThan1Value(4);
	Vector invalidNaturalCoordinateSize3(3), invalidNaturalCoordinateSize5(5);

	validNaturalCoordinate << 0.4, 0.3, 0.2, 0.1;
	validNaturalCoordinate2 << -1e-11, 1.0 + 1e-11, 0.0, 0.0;
	invalidNaturalCoordinateSumNot1 << 0.1, 0.1, 0.1, 0.1;
	invalidNaturalCoordinateNegativeValue << 0.7, 0.7, -0.5, 0.1;
	invalidNaturalCoordinateBiggerThan1Value << 1.4, 0.6, -1.2, 0.2;
	invalidNaturalCoordinateSize3 << 0.4, 0.4, 0.2;
	invalidNaturalCoordinateSize5 << 0.2, 0.2, 0.2, 0.2, 0.2;
	EXPECT_TRUE(element.isValidCoordinate(validNaturalCoordinate));
	EXPECT_TRUE(element.isValidCoordinate(validNaturalCoordinate2));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSumNot1));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateNegativeValue));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateBiggerThan1Value));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSize3));
	EXPECT_FALSE(element.isValidCoordinate(invalidNaturalCoordinateSize5));

	Vector naturalCoordinateA(4), naturalCoordinateB(4), naturalCoordinateC(4), naturalCoordinateD(4);
	Vector naturalCoordinateMiddle(4);
	Vector ptA, ptB, ptC, ptD, ptMiddle;
	naturalCoordinateA << 1.0, 0.0, 0.0, 0.0;
	naturalCoordinateB << 0.0, 1.0, 0.0, 0.0;
	naturalCoordinateC << 0.0, 0.0, 1.0, 0.0;
	naturalCoordinateD << 0.0, 0.0, 0.0, 1.0;
	naturalCoordinateMiddle << 1.0 / 4.0, 1.0 / 4.0, 1.0 / 4.0, 1 / 4.0;
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateBiggerThan1Value), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateNegativeValue), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSize3), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSize5), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(ptA = element.computeCartesianCoordinate(m_restState, invalidNaturalCoordinateSumNot1), \
				 SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(ptA = element.computeCartesianCoordinate(m_restState, naturalCoordinateA));
	EXPECT_NO_THROW(ptB = element.computeCartesianCoordinate(m_restState, naturalCoordinateB));
	EXPECT_NO_THROW(ptC = element.computeCartesianCoordinate(m_restState, naturalCoordinateC));
	EXPECT_NO_THROW(ptD = element.computeCartesianCoordinate(m_restState, naturalCoordinateD));
	EXPECT_NO_THROW(ptMiddle = element.computeCartesianCoordinate(m_restState, naturalCoordinateMiddle));
	EXPECT_TRUE(ptA.isApprox(expectedA));
	EXPECT_TRUE(ptB.isApprox(expectedB));
	EXPECT_TRUE(ptC.isApprox(expectedC));
	EXPECT_TRUE(ptD.isApprox(expectedD));
	EXPECT_TRUE(ptMiddle.isApprox((expectedA + expectedB + expectedC + expectedD) / 4.0));

	// Test computeNaturalCoordinate.
	SurgSim::Math::Vector2d cartesian2d(1, 0);
	SurgSim::Math::Vector4d cartesian4d(1, 0, 0, 0);
	EXPECT_THROW(element.computeNaturalCoordinate(m_restState, cartesian2d), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(element.computeNaturalCoordinate(m_restState, cartesian4d), SurgSim::Framework::AssertionFailure);

	std::vector<SurgSim::Math::Vector4d> listOfNaturalCoordinates;
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(1, 0, 0, 0));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0, 1, 0, 0));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0, 0, 1, 0));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0, 0, 0, 1));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0.354623, 0.768423, 0.12457, 0.327683));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(1.354623, 2.768423, 3.12457, 4.327683));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0.546323, 2.435323, -69.3422, 345.423));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0.352346, 3.3424, 9.325324, 5.32432));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0.623543, 9.2345, 2.45346, 1.645745));
	listOfNaturalCoordinates.push_back(SurgSim::Math::Vector4d(0.356234, 435.234, 32545.234, 9534.2123));

	SurgSim::Math::Vector4d input, calculated(0, 0, 0, 0);
	Vector cartesian;
	for (auto testCase = listOfNaturalCoordinates.begin(); testCase != listOfNaturalCoordinates.end(); ++testCase)
	{
		input = (*testCase).cwiseAbs();
		input /= input.sum();
		EXPECT_NO_THROW(cartesian = element.computeCartesianCoordinate(m_restState, input));
		EXPECT_NO_THROW(calculated = element.computeNaturalCoordinate(m_restState, cartesian););
		EXPECT_TRUE(input.isApprox(calculated));
	}
}

TEST_F(Fem3DElementTetrahedronTests, ShapeFunctionsTest)
{
	using SurgSim::Math::getSubVector;

	MockFem3DElementTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	EXPECT_TRUE(tet.getInitialPosition().isApprox(m_expectedX0)) <<
			"x0 = " << tet.getInitialPosition().transpose() << std::endl << "x0 expected = " <<
			m_expectedX0.transpose();

	double ai[4], bi[4], ci[4], di[4];
	double sumAi = 0.0;
	for (int i = 0; i < 4; i++)
	{
		tet.getShapeFunction(i, &(ai[i]), &(bi[i]), &(ci[i]), &(di[i]));
		sumAi += ai[i];
	}
	EXPECT_DOUBLE_EQ(6.0 * m_expectedVolume, sumAi);

	// Ni(x,y,z) = 1/6V . (ai + bi.x + ci.y + di.z)

	// We should have by construction:
	// { N0(p0) = 1    N1(p0)=N2(p0)=N3(p0)=0
	// { N1(p1) = 1    N1(p1)=N2(p1)=N3(p1)=0
	// { N2(p2) = 1    N1(p2)=N2(p2)=N3(p2)=0
	// { N3(p3) = 1    N1(p3)=N2(p3)=N3(p3)=0
	const Vector3d p0 = getSubVector(m_expectedX0, 0, 3);
	const Vector3d p1 = getSubVector(m_expectedX0, 1, 3);
	const Vector3d p2 = getSubVector(m_expectedX0, 2, 3);
	const Vector3d p3 = getSubVector(m_expectedX0, 3, 3);
	double Ni_p0[4], Ni_p1[4], Ni_p2[4], Ni_p3[4];
	for (int i = 0; i < 4; i++)
	{
		Ni_p0[i] = N(i, m_expectedVolume, ai, bi, ci, di, p0);
		Ni_p1[i] = N(i, m_expectedVolume, ai, bi, ci, di, p1);
		Ni_p2[i] = N(i, m_expectedVolume, ai, bi, ci, di, p2);
		Ni_p3[i] = N(i, m_expectedVolume, ai, bi, ci, di, p3);
	}
	EXPECT_NEAR(Ni_p0[0], 1.0, 1e-12);
	EXPECT_NEAR(Ni_p0[1], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p0[2], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p0[3], 0.0, 1e-12);

	EXPECT_NEAR(Ni_p1[0], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p1[1], 1.0, 1e-12);
	EXPECT_NEAR(Ni_p1[2], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p1[3], 0.0, 1e-12);

	EXPECT_NEAR(Ni_p2[0], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p2[1], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p2[2], 1.0, 1e-12);
	EXPECT_NEAR(Ni_p2[3], 0.0, 1e-12);

	EXPECT_NEAR(Ni_p3[0], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p3[1], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p3[2], 0.0, 1e-12);
	EXPECT_NEAR(Ni_p3[3], 1.0, 1e-12);

	// We should have the relation sum(Ni(x,y,z) = 1) for all points in the volume
	// We verify that relation by sampling the tetrahedron volume
	for (double sp0p1 = 0; sp0p1 <= 1.0; sp0p1 += 0.1)
	{
		for (double sp0p2 = 0; sp0p1 + sp0p2 <= 1.0; sp0p2 += 0.1)
		{
			for (double sp0p3 = 0; sp0p1 + sp0p2 + sp0p3 <= 1.0; sp0p3 += 0.1)
			{
				Vector3d p = p0 + sp0p1 * (p1 - p0) + sp0p2 * (p2 - p0) + sp0p3 * (p3 - p0);
				double Ni_p[4];
				for (int i = 0; i < 4; i++)
				{
					Ni_p[i] = N(i, m_expectedVolume, ai, bi, ci, di, p);
				}
				EXPECT_NEAR(Ni_p[0] + Ni_p[1] + Ni_p[2] + Ni_p[3], 1.0, 1e-10) <<
						" for sp0p1 = " << sp0p1 << ", sp0p2 = " << sp0p2 << ", sp0p3 = " << sp0p3 << std::endl <<
						" N0(x,y,z) = " << Ni_p[0] << " N1(x,y,z) = " << Ni_p[1] <<
						" N2(x,y,z) = " << Ni_p[2] << " N3(x,y,z) = " << Ni_p[3];
			}
		}
	}
}

TEST_F(Fem3DElementTetrahedronTests, ForceAndMatricesTest)
{
	using SurgSim::Math::getSubVector;

	MockFem3DElementTet tet(m_nodeIds);

	// Test the various mode of failure related to the physical parameters
	// This has been already tested in FemElementTests, but this is to make sure this method is called properly
	// So the same behavior should be expected
	{
		// Mass density not set
		ASSERT_ANY_THROW(tet.initialize(m_restState));

		// Poisson Ratio not set
		tet.setMassDensity(-1234.56);
		ASSERT_ANY_THROW(tet.initialize(m_restState));

		// Young modulus not set
		tet.setPoissonRatio(0.55);
		ASSERT_ANY_THROW(tet.initialize(m_restState));

		// Invalid mass density
		tet.setYoungModulus(-4321.33);
		ASSERT_ANY_THROW(tet.initialize(m_restState));

		// Invalid Poisson ratio
		tet.setMassDensity(m_rho);
		ASSERT_ANY_THROW(tet.initialize(m_restState));

		// Invalid Young modulus
		tet.setPoissonRatio(m_nu);
		ASSERT_ANY_THROW(tet.initialize(m_restState));

		tet.setYoungModulus(m_E);
		ASSERT_NO_THROW(tet.initialize(m_restState));
	}

	Vector forceVector = Vector::Zero(3 * 15);
	SparseMatrix massMatrix(3 * 15, 3 * 15);
	SparseMatrix dampingMatrix(3 * 15, 3 * 15);
	SparseMatrix stiffnessMatrix(3 * 15, 3 * 15);
	Matrix zeroMatrix = Matrix::Zero(tet.getNumDofPerNode() * tet.getNumNodes(),
									 tet.getNumDofPerNode() * tet.getNumNodes());
	massMatrix.setZero();
	tet.assembleMatrixBlocks(zeroMatrix, tet.getNodeIds(),
							 static_cast<SparseMatrix::Index>(tet.getNumDofPerNode()), &massMatrix, true);
	massMatrix.makeCompressed();
	dampingMatrix.setZero();
	tet.assembleMatrixBlocks(zeroMatrix, tet.getNodeIds(),
							 static_cast<SparseMatrix::Index>(tet.getNumDofPerNode()), &dampingMatrix, true);
	dampingMatrix.makeCompressed();
	stiffnessMatrix.setZero();
	tet.assembleMatrixBlocks(zeroMatrix, tet.getNodeIds(),
							 static_cast<SparseMatrix::Index>(tet.getNumDofPerNode()), &stiffnessMatrix, true);
	stiffnessMatrix.makeCompressed();

	// Make sure that the 2 ways of computing the expected stiffness matrix gives the same result
	EXPECT_TRUE(m_expectedStiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	// Update the internal f, M, D, K variables.
	tet.updateFMDK(m_restState, SurgSim::Math::ODEEQUATIONUPDATE_FMDK);

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	tet.addForce(&forceVector);
	EXPECT_TRUE(forceVector.isZero());

	tet.addMass(&massMatrix);
	EXPECT_TRUE(massMatrix.isApprox(m_expectedMassMatrix));

	tet.addDamping(&dampingMatrix);
	EXPECT_TRUE(dampingMatrix.isApprox(m_expectedDampingMatrix));

	tet.addStiffness(&stiffnessMatrix);
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	forceVector.setZero();
	clearMatrix(&massMatrix);
	clearMatrix(&dampingMatrix);
	clearMatrix(&stiffnessMatrix);

	tet.addFMDK(&forceVector, &massMatrix, &dampingMatrix, &stiffnessMatrix);
	EXPECT_TRUE(forceVector.isZero());
	EXPECT_TRUE(massMatrix.isApprox(m_expectedMassMatrix));
	EXPECT_TRUE(dampingMatrix.isApprox(m_expectedDampingMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	// Test addMatVec API with Mass component only
	forceVector.setZero();
	tet.addMatVec(1.0, 0.0, 0.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		EXPECT_NEAR(m_expectedMassMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Damping component only
	forceVector.setZero();
	tet.addMatVec(0.0, 1.0, 0.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		EXPECT_NEAR(m_expectedDampingMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Stiffness component only
	forceVector.setZero();
	tet.addMatVec(0.0, 0.0, 1.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		EXPECT_NEAR(m_expectedStiffnessMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with mix Mass/Damping/Stiffness components
	forceVector.setZero();
	tet.addMatVec(1.0, 2.0, 3.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		double expectedCoef = 1.0 * m_expectedMassMatrix.row(rowId).sum() +
							  2.0 * m_expectedDampingMatrix.row(rowId).sum() +
							  3.0 * m_expectedStiffnessMatrix.row(rowId).sum();
		EXPECT_NEAR(expectedCoef, forceVector[rowId], epsilon);
	}
}
