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

#include <SurgSim/Physics/FemElement3DTetrahedron.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Physics::FemElement3DTetrahedron;
using SurgSim::Physics::DeformableRepresentationState;
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
};

class MockFemElement3DTet : public FemElement3DTetrahedron
{
public:
	MockFemElement3DTet(std::array<unsigned int,4> nodeIds, const DeformableRepresentationState& restState) :
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

	const Vector& getF() const
	{
		return m_f;
	}
	const Matrix& getM() const
	{
		return m_M;
	}
	const Matrix& getD() const
	{
		return m_D;
	}
	const Matrix& getK() const
	{
		return m_K;
	}
};

class FemElement3DTetrahedronTests : public ::testing::Test
{
public:
	std::array<unsigned int, 4> m_nodeIds;
	DeformableRepresentationState m_restState;
	DeformableRepresentationState m_deformedState;
	double m_expectedVolume;
	Eigen::Matrix<double, 12, 1, Eigen::DontAlign> m_expectedX0;
	double m_rho, m_E, m_nu;
	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> m_expectedMassMatrix;
	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> m_expectedDampingMatrix;
	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> m_expectedStiffnessMatrix;
	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> m_expectedStiffnessMatrix2;

	virtual void SetUp() override
	{
		using SurgSim::Math::getSubVector;
		using SurgSim::Math::getSubMatrix;

		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;
		m_nodeIds[2] = 14;
		m_nodeIds[3] = 9;

		m_restState.setNumDof(3, 15);
		Vector& x0 = m_restState.getPositions();
		// Tet is aligned with the axis (X,Y,Z), centered on (0.1, 1.2, 2.3), embedded in a cube of size 1
		getSubVector(m_expectedX0, 0, 3) = getSubVector(x0, m_nodeIds[0], 3) = Vector3d(0.1, 1.2, 2.3);
		getSubVector(m_expectedX0, 1, 3) = getSubVector(x0, m_nodeIds[1], 3) = Vector3d(1.1, 1.2, 2.3);
		getSubVector(m_expectedX0, 2, 3) = getSubVector(x0, m_nodeIds[2], 3) = Vector3d(0.1, 2.2, 2.3);
		getSubVector(m_expectedX0, 3, 3) = getSubVector(x0, m_nodeIds[3], 3) = Vector3d(0.1, 1.2, 3.3);

		// The deformedState will be the rest state with an epsilon on each node
		m_deformedState = m_restState;
		Vector& x = m_deformedState.getPositions();
		getSubVector(x, m_nodeIds[0], 3) += Vector3d(0.1, 0.1, 0.1);
		getSubVector(x, m_nodeIds[1], 3) += Vector3d(0.1, 0.1, 0.1);
		getSubVector(x, m_nodeIds[2], 3) += Vector3d(0.1, 0.1, 0.1);
		getSubVector(x, m_nodeIds[3], 3) += Vector3d(0.1, 0.1, 0.1);


		// The tet is part of a cube of size 1x1x1 (it occupies 1/6 of the cube's volume)
		m_expectedVolume = 1.0 / 6.0;

		m_rho = 1000.0;
		m_E = 1e6;
		m_nu = 0.45;

		m_expectedMassMatrix.setZero();
		{
			m_expectedMassMatrix.diagonal().setConstant(2.0);
			m_expectedMassMatrix.block(0, 3, 9, 9).diagonal().setConstant(1.0);
			m_expectedMassMatrix.block(0, 6, 6, 6).diagonal().setConstant(1.0);
			m_expectedMassMatrix.block(0, 9, 3, 3).diagonal().setConstant(1.0);
			m_expectedMassMatrix.block(3, 0, 9, 9).diagonal().setConstant(1.0);
			m_expectedMassMatrix.block(6, 0, 6, 6).diagonal().setConstant(1.0);
			m_expectedMassMatrix.block(9, 0, 3, 3).diagonal().setConstant(1.0);
		}
		m_expectedMassMatrix *= m_rho * m_expectedVolume / 20.0;

		m_expectedDampingMatrix.setZero();

		m_expectedStiffnessMatrix.setZero();
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

			m_expectedStiffnessMatrix = m_expectedVolume * B.transpose() * E * B;
		}

		// Expecte stiffness matrix given for our case in:
		// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
		double E = m_E / (12.0*(1.0 - 2.0*m_nu)*(1.0 + m_nu));
		double n0 = 1.0 - 2.0 * m_nu;
		double n1 = 1.0 - m_nu;
		Eigen::Matrix<double, 12, 12, Eigen::DontAlign>& K = m_expectedStiffnessMatrix2;
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
	}
};

extern void testSize(const Vector& v, int expectedSize);
extern void testSize(const Matrix& m, int expectedRows, int expectedCols);

TEST_F(FemElement3DTetrahedronTests, ConstructorTest)
{
	ASSERT_NO_THROW({MockFemElement3DTet tet(m_nodeIds, m_restState);});
	{
		MockFemElement3DTet tet(m_nodeIds, m_restState);
		testSize(tet.getF(), 12);
		testSize(tet.getM(), 12, 12);
		testSize(tet.getD(), 12, 12);
		testSize(tet.getK(), 12, 12);
	}
	ASSERT_NO_THROW({MockFemElement3DTet* tet = new MockFemElement3DTet(m_nodeIds, m_restState); delete tet;});
	{
		MockFemElement3DTet* tet = new MockFemElement3DTet(m_nodeIds, m_restState);
		testSize(tet->getF(), 12);
		testSize(tet->getM(), 12, 12);
		testSize(tet->getD(), 12, 12);
		testSize(tet->getK(), 12, 12);
		delete tet;
	}
	ASSERT_NO_THROW({std::shared_ptr<MockFemElement3DTet> tet =
		std::make_shared<MockFemElement3DTet>(m_nodeIds, m_restState);});
	{
		std::shared_ptr<MockFemElement3DTet> tet = std::make_shared<MockFemElement3DTet>(m_nodeIds, m_restState);
		testSize(tet->getF(), 12);
		testSize(tet->getM(), 12, 12);
		testSize(tet->getD(), 12, 12);
		testSize(tet->getK(), 12, 12);
	}
}

TEST_F(FemElement3DTetrahedronTests, DefaultValueTest)
{
	FemElement3DTetrahedron tet(m_nodeIds, m_restState);

	EXPECT_DOUBLE_EQ(0.0, tet.getMassDensity());
	EXPECT_DOUBLE_EQ(0.0, tet.getYoungModulus());
	EXPECT_DOUBLE_EQ(0.0, tet.getPoissonRatio());
}

TEST_F(FemElement3DTetrahedronTests, SetGetTest)
{
	FemElement3DTetrahedron tet(m_nodeIds, m_restState);

	tet.setMassDensity(3.4);
	EXPECT_DOUBLE_EQ(3.4, tet.getMassDensity());
	tet.setYoungModulus(4534.33);
	EXPECT_DOUBLE_EQ(4534.33, tet.getYoungModulus());
	tet.setPoissonRatio(0.34);
	EXPECT_DOUBLE_EQ(0.34, tet.getPoissonRatio());
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
	EXPECT_NEAR(Ni_p0[0], 1.0, 1e-10);
	EXPECT_NEAR(Ni_p0[1], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p0[2], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p0[3], 0.0, 1e-10);

	EXPECT_NEAR(Ni_p1[0], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p1[1], 1.0, 1e-10);
	EXPECT_NEAR(Ni_p1[2], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p1[3], 0.0, 1e-10);

	EXPECT_NEAR(Ni_p2[0], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p2[1], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p2[2], 1.0, 1e-10);
	EXPECT_NEAR(Ni_p2[3], 0.0, 1e-10);

	EXPECT_NEAR(Ni_p3[0], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p3[1], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p3[2], 0.0, 1e-10);
	EXPECT_NEAR(Ni_p3[3], 1.0, 1e-10);

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
	tet.setMassDensity(m_rho);
	tet.setYoungModulus(m_E);
	tet.setPoissonRatio(m_nu);

	// Make sure that the 2 ways of computing the expected stiffness matrix gives the same result
	EXPECT_TRUE(m_expectedStiffnessMatrix.isApprox(m_expectedStiffnessMatrix2));

	// No force should be produced when in rest state (x = x0) => F = K.(x-x0) = 0
	EXPECT_TRUE(tet.computeForce(m_restState).isZero());
	EXPECT_TRUE(tet.computeMass(m_restState).isApprox(m_expectedMassMatrix));
	EXPECT_TRUE(tet.computeDamping(m_restState).isApprox(m_expectedDampingMatrix));
	EXPECT_TRUE(tet.computeStiffness(m_restState).isApprox(m_expectedStiffnessMatrix));
	EXPECT_TRUE(tet.computeStiffness(m_restState).isApprox(m_expectedStiffnessMatrix2));

	Vector *f;
	Matrix *M, *D, *K;
	tet.computeFMDK(m_restState, &f, &M, &D, &K);
	EXPECT_TRUE(f->isZero());
	EXPECT_TRUE(M->isApprox(m_expectedMassMatrix));
	EXPECT_TRUE(D->isApprox(m_expectedDampingMatrix));
	EXPECT_TRUE(K->isApprox(m_expectedStiffnessMatrix));
	EXPECT_TRUE(K->isApprox(m_expectedStiffnessMatrix2));
}
