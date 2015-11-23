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

#include <string>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/LinearSpring.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix66d;
using SurgSim::Math::SparseMatrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector6d;
using SurgSim::Physics::LinearSpring;

namespace
{
const double epsilon = 1e-10;
const double epsilonNumericalEvaluation = 1e-8;
const double epsilonTestAgainstNumericalApproximation = 1e-7;

Matrix66d KFormal(const Vector3d p0, const Vector3d p1,
				  const Vector3d v0, const Vector3d v1,
				  double l0, double stiffness, double damping)
{
	Vector3d u = p1 - p0;
	double m_l = u.norm();
	u /= m_l;
	double lRatio = (m_l - l0) / m_l;
	double vRatio = (v1 - v0).dot(u) / m_l;

	Matrix33d K00 = Matrix33d::Identity() * (stiffness * lRatio + damping * vRatio);
	K00 -= (u * u.transpose()) * (stiffness * (lRatio - 1.0) + 2.0 * damping * vRatio);
	K00 += damping * (u * (v1 - v0).transpose()) / m_l;

	Matrix66d K = Matrix66d::Zero();

	// Assembly stage in K
	SurgSim::Math::addSubMatrix(K00, 0, 0, 3, 3, &K);
	SurgSim::Math::addSubMatrix((-K00).eval(), 0, 1, 3, 3, &K);
	SurgSim::Math::addSubMatrix((-K00).eval(), 1, 0, 3, 3, &K);
	SurgSim::Math::addSubMatrix(K00, 1, 1, 3, 3, &K);

	return K;
}

Matrix66d DFormal(const Vector3d p0, const Vector3d p1,
				  const Vector3d v0, const Vector3d v1,
				  double l0, double stiffness, double damping)
{
	Vector3d u = p1 - p0;
	u.normalize();
	Matrix33d D00 = damping * (u * u.transpose());

	// Assembly stage in D
	Matrix66d D = Matrix66d::Zero();
	SurgSim::Math::addSubMatrix(D00, 0, 0, 3, 3, &D);
	SurgSim::Math::addSubMatrix((-D00).eval(), 0, 1, 3, 3, &D);
	SurgSim::Math::addSubMatrix((-D00).eval(), 1, 0, 3, 3, &D);
	SurgSim::Math::addSubMatrix(D00, 1, 1, 3, 3, &D);

	return D;
}

double f(size_t axis, const Vector3d p0, const Vector3d p1,
		 const Vector3d v0, const Vector3d v1,
		 double l0, double stiffness, double damping)
{
	Vector3d u = p1 - p0;
	double m_l = u.norm();
	u /= m_l;
	double elongationPosition = m_l - l0;
	double elongationVelocity = (v1 - v0).dot(u);
	if (axis <= 2)
	{
		return (stiffness * elongationPosition + damping * elongationVelocity) * u[axis];
	}
	else
	{
		return -(stiffness * elongationPosition + damping * elongationVelocity) * u[axis - 3];
	}
}

Matrix66d KNumerical(const Vector3d p0, const Vector3d p1,
					 const Vector3d v0, const Vector3d v1,
					 double l0, double stiffness, double damping)
{
	Matrix66d dfdx;

	for (size_t row = 0; row < 6; row++)
	{
		for (size_t col = 0; col < 6; col++)
		{
			// dfrow/dxcol
			// (1) f(x+delta) = f(x) + df/dx.delta + o(delta^2)
			// (2) f(x-delta) = f(x) - df/dx.delta + o(delta^2)
			// (1) - (2) f(x+delta) - f(x-delta) = 2df/dx.delta
			// df/dx = (f(x+delta) - f(x-delta)) / 2delta
			Vector6d delta6D = Vector6d::Zero();
			delta6D[col] = epsilonNumericalEvaluation;
			double f_plus_delta = f(row, p0 + delta6D.segment(0, 3), p1 + delta6D.segment(3, 3), v0, v1,
									l0, stiffness, damping);
			double f_minus_delta = f(row, p0 - delta6D.segment(0, 3), p1 - delta6D.segment(3, 3), v0, v1,
									 l0, stiffness, damping);
			dfdx(row, col) = (f_plus_delta - f_minus_delta) / (2.0 * epsilonNumericalEvaluation);
		}
	}

	return - dfdx;
}

Matrix66d DNumerical(const Vector3d p0, const Vector3d p1,
					 const Vector3d v0, const Vector3d v1,
					 double l0, double stiffness, double damping)
{
	Matrix66d dfdv;

	for (size_t row = 0; row < 6; row++)
	{
		for (size_t col = 0; col < 6; col++)
		{
			// dfrow/dxcol
			// (1) f(x+delta) = f(x) + df/dx.delta + o(delta^2)
			// (2) f(x-delta) = f(x) - df/dx.delta + o(delta^2)
			// (1) - (2) f(x+delta) - f(x-delta) = 2df/dx.delta
			// df/dx = (f(x+delta) - f(x-delta)) / 2delta
			Vector6d delta6D = Vector6d::Zero();
			delta6D[col] = epsilonNumericalEvaluation;
			double f_plus_delta = f(row, p0, p1, v0 + delta6D.segment(0, 3), v1 + delta6D.segment(3, 3),
									l0, stiffness, damping);
			double f_moins_delta = f(row, p0, p1, v0 - delta6D.segment(0, 3), v1 - delta6D.segment(3, 3),
									 l0, stiffness, damping);
			dfdv(row, col) = (f_plus_delta - f_moins_delta) / (2.0 * epsilonNumericalEvaluation);
		}
	}

	return - dfdv;
}

};

TEST(LinearSpringTests, Constructor)
{
	ASSERT_NO_THROW({LinearSpring ls(0, 1);});
	ASSERT_NO_THROW({LinearSpring* ls = new LinearSpring(0, 1); delete ls;});
	ASSERT_NO_THROW({std::shared_ptr<LinearSpring> ls = std::make_shared<LinearSpring>(0, 1);});
}

TEST(LinearSpringTests, SetGetMethods)
{
	LinearSpring ls(0, 1);

	// Stiffness getter/setter
	ASSERT_THROW(ls.setStiffness(-0.34), SurgSim::Framework::AssertionFailure);
	ASSERT_NO_THROW(ls.setStiffness(0.0));
	ASSERT_DOUBLE_EQ(0.0, ls.getStiffness());
	ASSERT_NO_THROW(ls.setStiffness(0.34));
	ASSERT_DOUBLE_EQ(0.34, ls.getStiffness());

	// Damping getter/setter
	ASSERT_THROW(ls.setDamping(-0.45), SurgSim::Framework::AssertionFailure);
	ASSERT_NO_THROW(ls.setDamping(0.0));
	ASSERT_DOUBLE_EQ(0.0, ls.getDamping());
	ASSERT_NO_THROW(ls.setDamping(0.45));
	ASSERT_DOUBLE_EQ(0.45, ls.getDamping());

	// Rest length getter/setter
	ASSERT_THROW(ls.setRestLength(-1.23), SurgSim::Framework::AssertionFailure);
	ASSERT_NO_THROW(ls.setRestLength(0.0));
	ASSERT_DOUBLE_EQ(0.0, ls.getRestLength());
	ASSERT_NO_THROW(ls.setRestLength(1.23));
	ASSERT_DOUBLE_EQ(1.23, ls.getRestLength());

	// Operator ==/!= (with same node Ids)
	LinearSpring ls2(0, 1);
	ASSERT_TRUE(ls != ls2);
	ls2.setStiffness(ls.getStiffness());
	ASSERT_TRUE(ls != ls2);
	ls2.setDamping(ls.getDamping());
	ASSERT_TRUE(ls != ls2);
	ls2.setRestLength(ls.getRestLength());
	ASSERT_TRUE(ls == ls2);
	ls2.setDamping(ls.getDamping() + 0.55);
	ASSERT_TRUE(ls != ls2);
	ls2.setDamping(ls.getDamping());
	ls2.setStiffness(ls.getStiffness() + 0.23);
	ASSERT_TRUE(ls != ls2);

	// Operator ==/!= (with different node Ids)
	LinearSpring ls3(0, 2);
	ASSERT_TRUE(ls != ls3);
	ls3.setStiffness(ls.getStiffness());
	ASSERT_TRUE(ls != ls3);
	ls3.setDamping(ls.getDamping());
	ASSERT_TRUE(ls != ls3);
	ls3.setRestLength(ls.getRestLength());
	ASSERT_TRUE(ls != ls3);
}

namespace
{
void initializeTest(bool setStiffness, bool setDamping, bool setRestLength, bool expectException)
{
	LinearSpring ls(0, 1);
	SurgSim::Math::OdeState state;
	std::string scopeTrace;

	state.setNumDof(3, 2);

	if (setStiffness)
	{
		ls.setStiffness(1.23);
		scopeTrace += "Stiffness set; ";
	}
	else
	{
		scopeTrace += "Stiffness unset; ";
	}

	if (setDamping)
	{
		ls.setDamping(1.23);
		scopeTrace += "Damping set; ";
	}
	else
	{
		scopeTrace += "Damping unset; ";
	}

	if (setRestLength)
	{
		ls.setRestLength(1.23);
		scopeTrace += "Rest length set; ";
	}
	else
	{
		scopeTrace += "Rest length unset; ";
	}

	SCOPED_TRACE(scopeTrace);
	if (expectException)
	{
		EXPECT_THROW(ls.initialize(state), SurgSim::Framework::AssertionFailure);
	}
	else
	{
		EXPECT_NO_THROW(ls.initialize(state));
	}
}
};
TEST(LinearSpringTests, initializeTest)
{
	//            (stiff  damp   restL| expectException)
	initializeTest(false, false, false, true);

	initializeTest(true, false, false, true);
	initializeTest(false, true, false, true);
	initializeTest(false, false, true, true);

	initializeTest(true, true, false, true);
	initializeTest(true, false, true, false);
	initializeTest(false, true, true, true);

	initializeTest(true, true, true, false);
}

TEST(LinearSpringTests, computeMethods)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Math::setSubMatrix;

	// Setup the spring
	LinearSpring ls(0, 1);
	ls.setStiffness(0.34);
	ls.setDamping(0.45);
	ls.setRestLength(1.23);

	// Setup the state
	SurgSim::Math::OdeState state;
	state.setNumDof(3u, 2u);
	setSubVector(Vector3d(0.0, 0.0, 0.0), 0, 3, &state.getPositions());
	setSubVector(Vector3d(2.3, 4.1, 1.2), 1, 3, &state.getPositions());
	setSubVector(Vector3d(0.1, 0.2, 0.5), 0, 3, &state.getVelocities());
	setSubVector(Vector3d(-0.3, -0.14, 0.0), 1, 3, &state.getVelocities());

	// Calculating spring force
	Vector3d expectedF3D(0.45563016177577925, 0.81221028838291076, 0.23772008440475439);
	Vector6d expectedF;
	setSubVector(expectedF3D, 0, 3, &expectedF);
	setSubVector(-expectedF3D, 1, 3, &expectedF);
	Vector f(6);
	f.setZero();
	ls.addForce(state, &f);
	EXPECT_TRUE(f.isApprox(expectedF)) << " F = " << f.transpose() << std::endl <<
									   "expectedF = " << expectedF.transpose() << std::endl;

	// Calculate stiffness matrix
	Matrix33d expectedStiffness33;
	expectedStiffness33 << 0.224919570941728960, 0.064210544149390564, 0.0011847965179350647,
						0.047808674990512064, 0.312562344690556770, 0.0021120285754494608,
						0.013992782924052311, 0.033501153469247251, 0.1987182250423049600;
	Matrix66d expectedK;
	setSubMatrix(expectedStiffness33, 0, 0, 3, 3, &expectedK);
	setSubMatrix(-expectedStiffness33, 0, 1, 3, 3, &expectedK);
	setSubMatrix(-expectedStiffness33, 1, 0, 3, 3, &expectedK);
	setSubMatrix(expectedStiffness33, 1, 1, 3, 3, &expectedK);

	Matrix zeroBlock = Matrix::Zero(6, 6);
	SparseMatrix K(6, 6);
	K.setZero();
	SurgSim::Math::addSubMatrix(zeroBlock, 0, 0, &K, true);
	K.makeCompressed();

	ls.addStiffness(state, &K);
	EXPECT_TRUE(K.isApprox(expectedK)) << " K = " << std::endl << K << std::endl <<
									   "expectedK = " << std::endl << expectedK << std::endl;

	// Calculate damping matrix
	Matrix33d expectedDamping33;
	expectedDamping33 << 0.101125743415463040, 0.180267629566694950, 0.052761257434154628,
					  0.180267629566694950, 0.321346644010195360, 0.094052676295666937,
					  0.052761257434154628, 0.094052676295666937, 0.027527612574341543;
	Matrix66d expectedD;
	setSubMatrix(expectedDamping33, 0, 0, 3, 3, &expectedD);
	setSubMatrix(-expectedDamping33, 0, 1, 3, 3, &expectedD);
	setSubMatrix(-expectedDamping33, 1, 0, 3, 3, &expectedD);
	setSubMatrix(expectedDamping33, 1, 1, 3, 3, &expectedD);

	SparseMatrix D(6, 6);
	D.setZero();
	SurgSim::Math::addSubMatrix(zeroBlock, 0, 0, &D, true);
	D.makeCompressed();

	ls.addDamping(state, &D);
	EXPECT_TRUE(D.isApprox(expectedD)) << " D = " << std::endl << D << std::endl <<
									   "expectedD = " << std::endl << expectedD << std::endl;

	// Compute all together
	{
		SCOPED_TRACE("Testing addFDK method call");
		Vector f = Vector::Zero(6);
		SurgSim::Math::clearMatrix(&K);
		SurgSim::Math::clearMatrix(&D);
		ls.addFDK(state, &f, &D, &K);
		EXPECT_TRUE(f.isApprox(expectedF)) << " F = " << f.transpose() << std::endl <<
										   "expectedF = " << expectedF.transpose() << std::endl;
		EXPECT_TRUE(K.isApprox(expectedK)) << " K = " << std::endl << K << std::endl <<
										   "expectedK = " << std::endl << expectedK << std::endl;
		EXPECT_TRUE(D.isApprox(expectedD)) << " D = " << std::endl << D << std::endl <<
										   "expectedD = " << std::endl << expectedD << std::endl;
	}

	// Test addMatVec method
	Vector ones = Vector::Ones(6);
	Vector oneToSix = Vector::LinSpaced(6, 1.0, 6.0);

	f.setZero();
	ls.addMatVec(state, 1.0, 0.0, ones, &f);
	{
		SCOPED_TRACE("addMatVec(..., dampingFactor=1, stiffnessFactor=0, (1 1 1 1 1 1),...)");
		for (size_t row = 0; row < 6; ++row)
		{
			EXPECT_NEAR(expectedD.row(row).sum(), f[row], epsilon) <<
					"f[" << row << "] = " << f[row] <<
					" expectedValue = " << expectedD.row(row).sum();
		}
	}
	f.setZero();
	ls.addMatVec(state, 1.0, 0.0, oneToSix, &f);
	{
		SCOPED_TRACE("addMatVec(..., dampingFactor=1, stiffnessFactor=0, (1 2 3 4 5 6),...)");
		for (size_t row = 0; row < 6; ++row)
		{
			EXPECT_NEAR(expectedD.row(row).dot(oneToSix), f[row], epsilon) <<
					"f[" << row << "] = " << f[row] <<
					" expectedValue = " << expectedD.row(row).dot(oneToSix);
		}
	}
	f.setZero();
	ls.addMatVec(state, 0.0, 1.0, ones, &f);
	{
		SCOPED_TRACE("addMatVec(..., dampingFactor=0, stiffnessFactor=1, (1 1 1 1 1 1),...)");
		for (size_t row = 0; row < 6; ++row)
		{
			EXPECT_NEAR(expectedK.row(row).sum(), f[row], epsilon) <<
					"f[" << row << "] = " << f[row] <<
					" expectedValue = " << expectedK.row(row).sum();
		}
	}
	f.setZero();
	ls.addMatVec(state, 0.0, 1.0, oneToSix, &f);
	{
		SCOPED_TRACE("addMatVec(..., dampingFactor=0, stiffnessFactor=1, (1 2 3 4 5 6),...)");
		for (size_t row = 0; row < 6; ++row)
		{
			EXPECT_NEAR(expectedK.row(row).dot(oneToSix), f[row], epsilon) <<
					"f[" << row << "] = " << f[row] <<
					" expectedValue = " << expectedK.row(row).dot(oneToSix);
		}
	}

	f.setZero();
	ls.addMatVec(state, 1.4, 4.1, ones, &f);
	{
		SCOPED_TRACE("addMatVec(..., dampingFactor=1.4, stiffnessFactor=4.1, (1 1 1 1 1 1),...)");
		for (size_t row = 0; row < 6; ++row)
		{
			EXPECT_NEAR(1.4 * expectedD.row(row).sum() + 4.1 * expectedK.row(row).sum(), f[row], epsilon) <<
					"f[" << row << "] = " << f[row] <<
					" expectedValue = " << 1.4 * expectedD.row(row).sum() + 4.1 * expectedK.row(row).sum();
		}
	}
	f.setZero();
	ls.addMatVec(state, 1.4, 4.1, oneToSix, &f);
	{
		SCOPED_TRACE("addMatVec(..., dampingFactor=1.4, stiffnessFactor=4.1, (1 2 3 4 5 6),...)");
		for (size_t row = 0; row < 6; ++row)
		{
			EXPECT_NEAR(\
						1.4 * expectedD.row(row).dot(oneToSix) + 4.1 * expectedK.row(row).dot(oneToSix),
						f[row], epsilon) <<
										 "f[" << row << "] = " << f[row] <<
										 " expectedValue = " << 1.4 * expectedD.row(row).dot(oneToSix) +
										 4.1 * expectedK.row(row).dot(oneToSix);
		}
	}
}

TEST(LinearSpringTests, addStiffnessNumericalTest)
{
	Vector3d x0(0.1, 0.2, 0.3), x1(1.3, 1.2, 1.45), v0(0.1, 0.2, 0.3), v1(-0.1, 0.12, -0.45);
	double restLength = 1.0, stiffness = 5.0, damping = 2.0;

	// Setup the spring
	LinearSpring ls(0, 1);
	ls.setStiffness(stiffness);
	ls.setDamping(damping);
	ls.setRestLength(restLength);

	// Setup the state
	SurgSim::Math::OdeState state;
	state.setNumDof(3, 2); // 2 nodes of 3DOF each
	state.getPositions().segment(0, 3) = x0;
	state.getPositions().segment(3, 3) = x1;
	state.getVelocities().segment(0, 3) = v0;
	state.getVelocities().segment(3, 3) = v1;

	Matrix zeroBlock = Matrix::Zero(6, 6);
	SparseMatrix K(6, 6);
	K.setZero();
	SurgSim::Math::addSubMatrix(zeroBlock, 0, 0, &K, true);
	K.makeCompressed();

	ls.addStiffness(state, &K);

	Eigen::Matrix<double, 6, 6> Knumeric = KNumerical(x0, x1, v0, v1, restLength, stiffness, damping);
	Eigen::Matrix<double, 6, 6> Kformal = KFormal(x0, x1, v0, v1, restLength, stiffness, damping);
	EXPECT_TRUE(Kformal.isApprox(Knumeric, epsilonTestAgainstNumericalApproximation)) << std::endl <<
			"Kformal = " << std::endl << Kformal << std::endl <<
			"Knumeric = " << std::endl << Knumeric << std::endl <<
			"Kformal - Knumeric= " << std::endl << Kformal - Knumeric << std::endl;
	EXPECT_TRUE(K.isApprox(Kformal)) << std::endl <<
									 "K = " << std::endl << K << std::endl <<
									 "Knumeric = " << std::endl << Knumeric << std::endl;
}

TEST(LinearSpringTests, addDampingNumericalTest)
{
	Vector3d x0(0.1, 0.2, 0.3), x1(1.3, 1.2, 1.45), v0(0.1, 0.2, 0.3), v1(-0.1, 0.12, -0.45);
	double restLength = 1.0, stiffness = 5.0, damping = 2.0;

	// Setup the spring
	LinearSpring ls(0, 1);
	ls.setStiffness(stiffness);
	ls.setDamping(damping);
	ls.setRestLength(restLength);

	// Setup the state
	SurgSim::Math::OdeState state;
	state.setNumDof(3, 2); // 2 nodes of 3DOF each
	state.getPositions().segment(0, 3) = x0;
	state.getPositions().segment(3, 3) = x1;
	state.getVelocities().segment(0, 3) = v0;
	state.getVelocities().segment(3, 3) = v1;

	Matrix zeroBlock = Matrix::Zero(6, 6);
	SparseMatrix D(6, 6);
	D.setZero();
	SurgSim::Math::addSubMatrix(zeroBlock, 0, 0, &D, true);
	D.makeCompressed();

	ls.addDamping(state, &D);

	Eigen::Matrix<double, 6, 6> Dnumeric = DNumerical(x0, x1, v0, v1, restLength, stiffness, damping);
	Eigen::Matrix<double, 6, 6> Dformal = DFormal(x0, x1, v0, v1, restLength, stiffness, damping);
	EXPECT_TRUE(Dformal.isApprox(Dnumeric, epsilonTestAgainstNumericalApproximation)) << std::endl <<
			"Dformal = " << std::endl << Dformal << std::endl <<
			"Dnumeric = " << std::endl << Dnumeric << std::endl;
	EXPECT_TRUE(D.isApprox(Dformal)) << std::endl <<
									 "D = " << std::endl << D << std::endl <<
									 "Dnumeric = " << std::endl << Dnumeric << std::endl;
}

TEST(LinearSpringTests, addFDKNumericalTest)
{
	Vector3d x0(0.1, 0.2, 0.3), x1(1.3, 1.2, 1.45), v0(0.1, 0.2, 0.3), v1(-0.1, 0.12, -0.45);
	double restLength = 1.0, stiffness = 5.0, damping = 2.0;

	// Setup the spring
	LinearSpring ls(0, 1);
	ls.setStiffness(stiffness);
	ls.setDamping(damping);
	ls.setRestLength(restLength);

	// Setup the state
	SurgSim::Math::OdeState state;
	state.setNumDof(3, 2); // 2 nodes of 3DOF each
	state.getPositions().segment(0, 3) = x0;
	state.getPositions().segment(3, 3) = x1;
	state.getVelocities().segment(0, 3) = v0;
	state.getVelocities().segment(3, 3) = v1;

	Vector F = Vector::Zero(6);
	Matrix zeroBlock = Matrix::Zero(6, 6);
	SparseMatrix K(6, 6);
	K.setZero();
	SurgSim::Math::addSubMatrix(zeroBlock, 0, 0, &K, true);
	K.makeCompressed();

	SparseMatrix D(6, 6);
	D.setZero();
	SurgSim::Math::addSubMatrix(zeroBlock, 0, 0, &D, true);
	D.makeCompressed();

	ls.addFDK(state, &F, &D, &K);

	Eigen::Matrix<double, 6, 6> Knumeric = KNumerical(x0, x1, v0, v1, restLength, stiffness, damping);
	Eigen::Matrix<double, 6, 6> Kformal = KFormal(x0, x1, v0, v1, restLength, stiffness, damping);
	EXPECT_TRUE(Kformal.isApprox(Knumeric, epsilonTestAgainstNumericalApproximation)) << std::endl <<
			"Kformal = " << std::endl << Kformal << std::endl <<
			"Knumeric = " << std::endl << Knumeric << std::endl <<
			"Kformal - Knumeric= " << std::endl << Kformal - Knumeric << std::endl;
	EXPECT_TRUE(K.isApprox(Kformal)) << std::endl <<
									 "K = " << std::endl << K << std::endl <<
									 "Knumeric = " << std::endl << Knumeric << std::endl;

	Eigen::Matrix<double, 6, 6> Dnumeric = DNumerical(x0, x1, v0, v1, restLength, stiffness, damping);
	Eigen::Matrix<double, 6, 6> Dformal = DFormal(x0, x1, v0, v1, restLength, stiffness, damping);
	EXPECT_TRUE(Dformal.isApprox(Dnumeric, epsilonTestAgainstNumericalApproximation)) << std::endl <<
			"Dformal = " << std::endl << Dformal << std::endl <<
			"Dnumeric = " << std::endl << Dnumeric << std::endl <<
			"Dformal - Dnumeric= " << std::endl << Dformal - Dnumeric << std::endl;
	EXPECT_TRUE(D.isApprox(Dformal)) << std::endl <<
									 "D = " << std::endl << D << std::endl <<
									 "Dnumeric = " << std::endl << Dnumeric << std::endl;
}
