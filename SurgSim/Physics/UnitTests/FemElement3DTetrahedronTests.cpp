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

#include "SurgSim/Physics/FemElement3DTetrahedron.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"

using SurgSim::Physics::FemElement3DTetrahedron;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;

namespace
{
/// Useful function to compute a shape function
/// \param i ith shape function
/// \param ai, bi, ci, di The shape functions parameters
/// \param p The point to evaluate the shape function at
/// \return Ni(p) = 1/6V . (ai + bi.px + ci.py + di.pz)
double N(unsigned int i, double V, double *ai, double *bi, double *ci, double *di, const Vector3d& p)
{
	double inv6V = 1.0 / (6.0 * V);
	return inv6V * (ai[i] + bi[i] * p[0] + ci[i] * p[1] + di[i] * p[2]);
}

const double epsilon = 1e-9;
};

class MockFemElement3DTet : public FemElement3DTetrahedron
{
public:
	MockFemElement3DTet(std::array<unsigned int, 4> nodeIds, const DeformableRepresentationState& restState) :
		FemElement3DTetrahedron(nodeIds, restState)
	{
	}

	double getRestVolume() const
	{
		return m_restVolume;
	}

	void getShapeFunction(int i, double *ai, double *bi, double *ci, double *di) const
	{
		*ai = m_ai[i];
		*bi = m_bi[i];
		*ci = m_ci[i];
		*di = m_di[i];
	}

	const Eigen::Matrix<double, 12, 1, Eigen::DontAlign>& getInitialPosition() const
	{
		return m_x0;
	}
};

class FemElement3DTetrahedronTests : public ::testing::Test
{
public:
	std::array<unsigned int, 4> m_nodeIds;
	DeformableRepresentationState m_restState;
	double m_expectedVolume;
	Eigen::Matrix<double, 12, 1, Eigen::DontAlign> m_expectedX0;
	double m_rho, m_E, m_nu;
	SurgSim::Math::Matrix m_expectedMassMatrix, m_expectedDampingMatrix;
	SurgSim::Math::Matrix m_expectedStiffnessMatrix, m_expectedStiffnessMatrix2;
	SurgSim::Math::Vector m_vectorOnes;

	virtual void SetUp() override
	{
		using SurgSim::Math::getSubVector;
		using SurgSim::Math::getSubMatrix;
		using SurgSim::Math::addSubMatrix;

		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;
		m_nodeIds[2] = 14;
		m_nodeIds[3] = 9;
		std::vector<unsigned int> m_nodeIdsVectorForm; // Useful for assembly helper function
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

		m_expectedMassMatrix.resize(3*15, 3*15);
		m_expectedMassMatrix.setZero();
		m_expectedDampingMatrix.resize(3*15, 3*15);
		m_expectedDampingMatrix.setZero();
		m_expectedStiffnessMatrix.resize(3*15, 3*15);
		m_expectedStiffnessMatrix.setZero();
		m_expectedStiffnessMatrix2.resize(3*15, 3*15);
		m_expectedStiffnessMatrix2.setZero();
		m_vectorOnes.resize(3*15);
		m_vectorOnes.setConstant(1.0);

		Eigen::Matrix<double, 12, 12, Eigen::DontAlign> M;
		M.setZero();
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

		m_expectedDampingMatrix.setZero();

		Eigen::Matrix<double, 12, 12, Eigen::DontAlign> K;
		K.setZero();
		{
			// Calculation done by hand from
			// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
			// ai = {}
			// bi = {-1 1 0 0}
			// ci = {-1 0 1 0}
			// di = {-1 0 0 1}
			Eigen::Matrix<double, 6, 12, Eigen::DontAlign> B;
			Eigen::Matrix<double, 6, 6, Eigen::DontAlign> E;

			B.setZero();
			B(0, 0) = -1; B(0, 3) = 1;
			B(1, 1) = -1; B(1, 7) = 1;
			B(2, 2) = -1; B(2, 11) = 1;
			B(3, 0) = -1; B(3, 1) = -1;  B(3, 4) = 1; B(3, 6) = 1;
			B(4, 1) = -1; B(4, 2) = -1;  B(4, 8) = 1; B(4, 10) = 1;
			B(5, 0) = -1; B(5, 2) = -1;  B(5, 5) = 1; B(5, 9) = 1;
			B *= 1.0 / (6.0 * m_expectedVolume);

			E.setZero();
			E.block(0, 0, 3, 3).setConstant(m_nu);
			E.block(0, 0, 3, 3).diagonal().setConstant(1.0 - m_nu);
			E.block(3, 3, 3, 3).diagonal().setConstant(0.5 - m_nu);
			E *= m_E / (( 1.0 + m_nu) * (1.0 - 2.0 * m_nu));

			K = m_expectedVolume * B.transpose() * E * B;
		}
		addSubMatrix(K, m_nodeIdsVectorForm, 3 , &m_expectedStiffnessMatrix);

		// Expecte stiffness matrix given for our case in:
		// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
		double E = m_E / (12.0*(1.0 - 2.0*m_nu)*(1.0 + m_nu));
		double n0 = 1.0 - 2.0 * m_nu;
		double n1 = 1.0 - m_nu;
		K.setZero();

		// Fill up the upper triangle part first (without diagonal elements)
		K(0, 1) = K(0, 2) = K(1, 2) = 1.0;

		K(0, 3) = -2.0 * n1;   K(0, 4) = -n0; K(0, 5) = -n0;
		K(1, 3) = -2.0 * m_nu; K(1, 4) = -n0;
		K(2, 3) = -2.0 * m_nu; K(2, 5) = -n0;

		K(0, 6) = - n0; K(0, 7) = -2.0 * m_nu;
		K(1, 6) = - n0; K(1, 7) = -2.0 * n1; K(1, 8) = - n0;
		K(2, 7) = - 2.0 * m_nu; K(2, 8) = -n0;

		K(0, 9) = - n0; K(0, 11) = -2.0 * m_nu;
		K(1, 10) = - n0; K(1, 11) = -2.0 * m_nu;
		K(2, 9) = - n0; K(2, 10) = - n0; K(2, 11) = -2.0 * n1;

		K(3, 7) = K(3, 11) =  2.0 * m_nu;
		K(4, 6) = n0;
		K(5, 9) = n0;
		K(7, 11) = 2.0 * m_nu;
		K(8, 10) = n0;

		K += K.transpose().eval(); // symmetric part (do not forget the .eval() !)

		K.block(0,0,3,3).diagonal().setConstant(4.0 - 6.0 * m_nu); // diagonal elements
		K.block(3,3,9,9).diagonal().setConstant(n0); // diagonal elements
		K(3, 3) = K(7, 7) = K(11, 11) = 2.0 * n1; // diagonal elements

		K *= E;

		addSubMatrix(K, m_nodeIdsVectorForm, 3 , &m_expectedStiffnessMatrix2);
	}
};

extern void testSize(const Vector& v, int expectedSize);
extern void testSize(const Matrix& m, int expectedRows, int expectedCols);

TEST_F(FemElement3DTetrahedronTests, ConstructorTest)
{
	ASSERT_NO_THROW({MockFemElement3DTet tet(m_nodeIds, m_restState);});
	ASSERT_NO_THROW({MockFemElement3DTet* tet = new MockFemElement3DTet(m_nodeIds, m_restState); delete tet;});
	ASSERT_NO_THROW({std::shared_ptr<MockFemElement3DTet> tet =
		std::make_shared<MockFemElement3DTet>(m_nodeIds, m_restState);});
}

TEST_F(FemElement3DTetrahedronTests, NodeIdsTest)
{
	FemElement3DTetrahedron tet(m_nodeIds, m_restState);
	EXPECT_EQ(4u, tet.getNumNodes());
	EXPECT_EQ(4u, tet.getNodeIds().size());
	for (int i = 0; i < 4; i++)
	{
		EXPECT_EQ(m_nodeIds[i], tet.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], tet.getNodeIds()[i]);
	}
}

TEST_F(FemElement3DTetrahedronTests, VolumeTest)
{
	MockFemElement3DTet tet(m_nodeIds, m_restState);
	EXPECT_NEAR(tet.getRestVolume(), m_expectedVolume, 1e-10);
	EXPECT_NEAR(tet.getVolume(m_restState), m_expectedVolume, 1e-10);
}

TEST_F(FemElement3DTetrahedronTests, ShapeFunctionsTest)
{
	using SurgSim::Math::getSubVector;

	MockFemElement3DTet tet(m_nodeIds, m_restState);

	EXPECT_TRUE(tet.getInitialPosition().isApprox(m_expectedX0)) <<
		"x0 = " << tet.getInitialPosition().transpose() << std::endl << "x0 expected = " << m_expectedX0.transpose();

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
	for (double sp0p1 = 0; sp0p1 <= 1.0; sp0p1+=0.1)
	{
		for (double sp0p2 = 0; sp0p1 + sp0p2 <= 1.0; sp0p2+=0.1)
		{
			for (double sp0p3 = 0; sp0p1 + sp0p2 + sp0p3 <= 1.0; sp0p3+=0.1)
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

TEST_F(FemElement3DTetrahedronTests, ForceAndMatricesTest)
{
	using SurgSim::Math::getSubVector;

	MockFemElement3DTet tet(m_nodeIds, m_restState);

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

	SurgSim::Math::Vector forceVector(3*15);
	SurgSim::Math::Matrix massMatrix(3*15, 3*15);
	SurgSim::Math::Matrix dampingMatrix(3*15, 3*15);
	SurgSim::Math::Matrix stiffnessMatrix(3*15, 3*15);

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	// Make sure that the 2 ways of computing the expected stiffness matrix gives the same result
	EXPECT_TRUE(m_expectedStiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	tet.addForce(m_restState, &forceVector);
	EXPECT_TRUE(forceVector.isZero());

	tet.addMass(m_restState, &massMatrix);
	EXPECT_TRUE(massMatrix.isApprox(m_expectedMassMatrix));

	tet.addDamping(m_restState, &dampingMatrix);
	EXPECT_TRUE(dampingMatrix.isApprox(m_expectedDampingMatrix));

	tet.addStiffness(m_restState, &stiffnessMatrix);
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	forceVector.setZero();
	massMatrix.setZero();
	dampingMatrix.setZero();
	stiffnessMatrix.setZero();

	tet.addFMDK(m_restState, &forceVector, &massMatrix, &dampingMatrix, &stiffnessMatrix);
	EXPECT_TRUE(forceVector.isZero());
	EXPECT_TRUE(massMatrix.isApprox(m_expectedMassMatrix));
	EXPECT_TRUE(dampingMatrix.isApprox(m_expectedDampingMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix));
	EXPECT_TRUE(stiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	// Test addMatVec API with Mass component only
	forceVector.setZero();
	tet.addMatVec(m_restState, 1.0, 0.0, 0.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		EXPECT_NEAR(m_expectedMassMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Damping component only
	forceVector.setZero();
	tet.addMatVec(m_restState, 0.0, 1.0, 0.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		EXPECT_NEAR(m_expectedDampingMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with Stiffness component only
	forceVector.setZero();
	tet.addMatVec(m_restState, 0.0, 0.0, 1.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		EXPECT_NEAR(m_expectedStiffnessMatrix.row(rowId).sum(), forceVector[rowId], epsilon);
	}
	// Test addMatVec API with mix Mass/Damping/Stiffness components
	forceVector.setZero();
	tet.addMatVec(m_restState, 1.0, 2.0, 3.0, m_vectorOnes, &forceVector);
	for (int rowId = 0; rowId < 3 * 15; rowId++)
	{
		double expectedCoef = 1.0 * m_expectedMassMatrix.row(rowId).sum() +
			2.0 * m_expectedDampingMatrix.row(rowId).sum() +
			3.0 * m_expectedStiffnessMatrix.row(rowId).sum();
		EXPECT_NEAR(expectedCoef, forceVector[rowId], epsilon);
	}
}
