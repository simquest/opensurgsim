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

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"

using SurgSim::Physics::Fem3DElementCorotationalTetrahedron;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix;
using SurgSim::Math::Matrix33d;

namespace
{
const double epsilonAddForce = 1e-8;
const double epsilonAddMatVec = 1e-10;
};

class MockFem3DElementCorotationalTet : public Fem3DElementCorotationalTetrahedron
{
public:
	explicit MockFem3DElementCorotationalTet(std::array<size_t, 4> nodeIds)
		: Fem3DElementCorotationalTetrahedron(nodeIds)
	{
	}

	const Eigen::Matrix<double,12 ,1>& getInitialPosition() const
	{
		return m_x0;
	}

	const Eigen::Matrix<double, 12, 12>& getNonRotatedMassMatrix() const
	{
		return m_M;
	}

	const Eigen::Matrix<double, 12, 12>& getNonRotatedStiffnessMatrix() const
	{
		return m_K;
	}

	const Eigen::Matrix<double, 12, 12> getRotatedStiffnessMatrix(const SurgSim::Math::OdeState& state) const
	{
		Eigen::Matrix<double, 12, 12> RKRt;
		computeRotationMassAndStiffness(state, nullptr, nullptr, &RKRt);
		return RKRt;
	}

	const Eigen::Matrix<double, 12, 12> getRotatedMassMatrix(const SurgSim::Math::OdeState& state) const
	{
		Eigen::Matrix<double, 12, 12> RMRt;
		computeRotationMassAndStiffness(state, nullptr, &RMRt, nullptr);
		return RMRt;
	}

	/// Compute the rotation, mass and stiffness matrices of the element from the given state
	/// \param state The state to compute the rotation and jacobians from
	/// \param [out] R rotation matrix of the element in the given state (can be nullptr if not needed)
	/// \param [out] Me, Ke Respectively the mass and stiffness matrices (Me and/or Ke be nullptr if not needed)
	/// \note The model is not viscoelastic but purely elastic, so there is no damping matrix here.
	void computeRotationMassAndStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix33d* R,
		Eigen::Matrix<double, 12, 12>* Me, Eigen::Matrix<double, 12, 12>* Ke) const
	{
		Fem3DElementCorotationalTetrahedron::computeRotationMassAndStiffness(state, R, Me, Ke);
	}

	const SurgSim::Math::Matrix33d getRotation(const SurgSim::Math::OdeState& state) const
	{
		SurgSim::Math::Matrix33d R;
		computeRotationMassAndStiffness(state, &R, nullptr, nullptr);
		return R;
	}

	const SurgSim::Math::Matrix44d getVInverse() const
	{
		return m_Vinverse;
	}

	void setupInitialParams(const SurgSim::Math::OdeState &state,
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

class Fem3DElementCorotationalTetrahedronTests : public ::testing::Test
{
public:
	std::array<size_t, 4> m_nodeIds;
	std::vector<size_t> m_nodeIdsAsVector;
	SurgSim::Math::OdeState m_restState, m_invalidState, m_state;
	double m_rho, m_E, m_nu;

	SurgSim::Math::Matrix33d m_rotation;
	Eigen::Matrix<double, 12, 12> m_R12x12;
	Vector3d m_translation;

	void SetUp() override
	{
		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;
		m_nodeIds[2] = 14;
		m_nodeIds[3] = 9;
		m_nodeIdsAsVector.assign(m_nodeIds.cbegin(), m_nodeIds.cend());

		m_restState.setNumDof(3, 15);
		m_invalidState.setNumDof(3, 15);
		Vector& x0 = m_restState.getPositions();
		Vector& invalidx0 = m_invalidState.getPositions();
		std::array<Vector3d, 4> points = {{Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0),
			Vector3d(0.0, 1.0, 0.0), Vector3d(0.0, 0.0, 1.0) }};

		// Tet is aligned with the axis (X,Y,Z), centered on (0.0, 0.0, 0.0), embedded in a cube of size 1
		for (size_t nodeId = 0; nodeId < 4; ++nodeId)
		{
			SurgSim::Math::getSubVector(x0, m_nodeIds[nodeId], 3) = points[nodeId];
			SurgSim::Math::getSubVector(invalidx0, m_nodeIds[nodeId], 3) = points[nodeId];
		}
		// In the invalid state, the tetrahedron is degenerated to a triangle (last 2 points are equal)
		SurgSim::Math::getSubVector(invalidx0, m_nodeIds[3], 3) = points[2];

		m_rho = 1000.0;
		m_E = 1e6;
		m_nu = 0.45;

		Vector3d axis(1.0, 0.2, -0.3);
		axis.normalize();
		m_rotation = SurgSim::Math::makeRotationMatrix(4.1415, axis);
		m_R12x12 = Eigen::Matrix<double, 12, 12>::Zero();
		for (size_t nodeId = 0; nodeId < 4; ++nodeId)
		{
			m_R12x12.block<3, 3>(3 * nodeId, 3 * nodeId) = m_rotation;
		}
		m_translation = Vector3d(1.2, 2.3, 3.4);
	}
};

namespace
{
template <class T, int MOpt>
void defineCurrentState(const SurgSim::Math::OdeState& x0, SurgSim::Math::OdeState* x,
						const Eigen::Transform<T, 3, MOpt>& t, bool addSmallDeformation)
{
	std::array<Vector3d, 3> delta = {{Vector3d(0.01, -0.02, 0.005),
		Vector3d(-0.01, -0.01, -0.03), Vector3d(0.0, -0.015, 0.03)}};

	*x = x0;
	for (size_t nodeId = 0; nodeId < x0.getNumNodes(); ++nodeId)
	{
		(*x).getPositions().segment<3>(3 * nodeId) = t * x0.getPositions().segment<3>(3 * nodeId);
		if (addSmallDeformation)
		{
			(*x).getPositions().segment<3>(3 * nodeId) += delta[nodeId % 3];
		}
	}
}
// Duplicates 4 times a 3x3 matrix on the diagonal blocks
Eigen::Matrix<double, 12, 12> make12x12(const Eigen::Matrix<double, 3, 3>& R)
{
	Eigen::Matrix<double, 12, 12> res = Eigen::Matrix<double, 12, 12>::Zero();
	for (size_t nodeId = 0; nodeId < 4; nodeId++)
	{
		res.block<3, 3>(3 * nodeId, 3 * nodeId) = R;
	}
	return res;
}
// Duplicates 4 times a 3x3 matrix on the diagonal blocks, with the 3x3 row-major matrix stored in a vector
Eigen::Matrix<double, 12, 12> make12x12(const Eigen::Matrix<double, 9, 1>& R)
{
	return make12x12(Eigen::Matrix<double, 3, 3>(R.data()));
}
}; // anonymous namespace

TEST_F(Fem3DElementCorotationalTetrahedronTests, ConstructorTest)
{
	ASSERT_NO_THROW({MockFem3DElementCorotationalTet tet(m_nodeIds);});
	ASSERT_NO_THROW({MockFem3DElementCorotationalTet* tet = new MockFem3DElementCorotationalTet(m_nodeIds);
		delete tet;});
	ASSERT_NO_THROW({std::shared_ptr<MockFem3DElementCorotationalTet> tet =
		std::make_shared<MockFem3DElementCorotationalTet>(m_nodeIds);});
}

TEST_F(Fem3DElementCorotationalTetrahedronTests, InitializeTest)
{
	MockFem3DElementCorotationalTet tet(m_nodeIds);
	ASSERT_NO_THROW(tet.setupInitialParams(m_restState, m_rho, m_nu, m_E));

	EXPECT_TRUE(tet.getRotation(m_restState).isIdentity());
	EXPECT_TRUE(tet.getRotatedStiffnessMatrix(m_restState).isApprox(tet.getNonRotatedStiffnessMatrix()));
	EXPECT_TRUE(tet.getRotatedMassMatrix(m_restState).isApprox(tet.getNonRotatedMassMatrix()));

	// V^1 = (a b c d)^-1
	//       (1 1 1 1)
	SurgSim::Math::Matrix44d expectedV = SurgSim::Math::Matrix44d::Ones();
	for (size_t axis = 0; axis < 3; ++axis)
	{
		expectedV(axis, 0) = m_restState.getPosition(m_nodeIds[0])(axis);
		expectedV(axis, 1) = m_restState.getPosition(m_nodeIds[1])(axis);
		expectedV(axis, 2) = m_restState.getPosition(m_nodeIds[2])(axis);
		expectedV(axis, 3) = m_restState.getPosition(m_nodeIds[3])(axis);
	}
	EXPECT_TRUE(tet.getVInverse().isApprox(expectedV.inverse()));

	MockFem3DElementCorotationalTet invalidTet(m_nodeIds);
	ASSERT_THROW(invalidTet.setupInitialParams(m_invalidState, m_rho, m_nu, m_E),
		SurgSim::Framework::AssertionFailure);
}

TEST_F(Fem3DElementCorotationalTetrahedronTests, ComputeRotationMassAndStiffnessTest)
{
	using SurgSim::Math::skew;
	using SurgSim::Math::makeSkewSymmetricMatrix;

	Eigen::Transform<double, 3, Eigen::Affine> transformation;
	SurgSim::Math::Matrix33d R;
	Eigen::Matrix<double, 12, 12> M, K;

	{
		SCOPED_TRACE("No rotation, no translation");
		transformation.linear().setIdentity();
		transformation.translation().setZero();
		defineCurrentState(m_restState, &m_state, transformation, false);

		MockFem3DElementCorotationalTet tet(m_nodeIds);
		tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

		ASSERT_NO_THROW(tet.computeRotationMassAndStiffness(m_state, &R, &M, &K));
		EXPECT_TRUE(R.isIdentity());
		EXPECT_TRUE(K.isApprox(tet.getNonRotatedStiffnessMatrix()));
		EXPECT_TRUE(M.isApprox(tet.getNonRotatedMassMatrix()));
	}

	{
		SCOPED_TRACE("Pure translation");
		transformation.linear().setIdentity();
		transformation.translation() = m_translation;
		defineCurrentState(m_restState, &m_state, transformation, false);

		MockFem3DElementCorotationalTet tet(m_nodeIds);
		tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

		ASSERT_NO_THROW(tet.computeRotationMassAndStiffness(m_state, &R, &M, &K));
		EXPECT_TRUE(R.isIdentity());
		EXPECT_TRUE(K.isApprox(tet.getNonRotatedStiffnessMatrix()));
		EXPECT_TRUE(M.isApprox(tet.getNonRotatedMassMatrix()));
	}

	{
		SCOPED_TRACE("Pure rotation");
		transformation.linear() = m_rotation;
		transformation.translation().setZero();
		defineCurrentState(m_restState, &m_state, transformation, false);

		MockFem3DElementCorotationalTet tet(m_nodeIds);
		tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

		ASSERT_NO_THROW(tet.computeRotationMassAndStiffness(m_state, &R, &M, &K));
		EXPECT_TRUE(R.isApprox(m_rotation));
		// The corotational stiffness has more terms than R.K0.R^t
		// But, these terms are in the order of epsilon is no scaling is involved in the transformation.
		// They seem to account for the deformation part of the transformation and not the rigid transformation.
		// A pure transformation (rotation + translation) is fully taken into account by the term R.K0.R^t
		EXPECT_TRUE(K.isApprox(m_R12x12 * tet.getNonRotatedStiffnessMatrix() * m_R12x12.transpose()));
		EXPECT_FALSE(K.isApprox(tet.getNonRotatedStiffnessMatrix()));
		EXPECT_TRUE(M.isApprox(m_R12x12 * tet.getNonRotatedMassMatrix() * m_R12x12.transpose()));
	}

	{
		SCOPED_TRACE("Translation + Pure Rotation");
		transformation.linear() = m_rotation;
		transformation.translation() = m_translation;
		defineCurrentState(m_restState, &m_state, transformation, false);

		MockFem3DElementCorotationalTet tet(m_nodeIds);
		tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

		ASSERT_NO_THROW(tet.computeRotationMassAndStiffness(m_state, &R, &M, &K));
		EXPECT_TRUE(R.isApprox(m_rotation));
		// The corotational stiffness has more terms than R.K0.R^t
		// But, these terms are in the order of epsilon is no scaling is involved in the transformation.
		// They seem to account for the deformation part of the transformation and not the rigid transformation.
		// A pure transformation (rotation + translation) is fully taken into account by the term R.K0.R^t
		EXPECT_TRUE(K.isApprox(m_R12x12 * tet.getNonRotatedStiffnessMatrix() * m_R12x12.transpose()));
		EXPECT_FALSE(K.isApprox(tet.getNonRotatedStiffnessMatrix()));
		EXPECT_TRUE(M.isApprox(m_R12x12 * tet.getNonRotatedMassMatrix() * m_R12x12.transpose()));
	}

	{
		SCOPED_TRACE("Affine transform : Translation + rotation * scaling");
		Eigen::UniformScaling<double> S = Eigen::Scaling(45.3);
		transformation.linear() = m_rotation * S;
		transformation.translation() = m_translation;
		defineCurrentState(m_restState, &m_state, transformation, false);

		MockFem3DElementCorotationalTet tet(m_nodeIds);
		tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

		ASSERT_NO_THROW(tet.computeRotationMassAndStiffness(m_state, &R, &M, &K));
		EXPECT_TRUE(R.isApprox(m_rotation));
		// The corotational stiffness has more terms than R.K0.R^t
		// But, these terms are in the order of epsilon is no scaling is involved in the transformation.
		// They seem to account for the deformation part of the transformation and not the rigid transformation.
		// A pure transformation (rotation + translation) is fully taken into account by the term R.K0.R^t
		EXPECT_FALSE(K.isApprox(m_R12x12 * tet.getNonRotatedStiffnessMatrix() * m_R12x12.transpose()));
		EXPECT_FALSE(K.isApprox(tet.getNonRotatedStiffnessMatrix()));
		EXPECT_TRUE(M.isApprox(m_R12x12 * tet.getNonRotatedMassMatrix() * m_R12x12.transpose()));
	}
}

namespace
{
void testAddStiffness(MockFem3DElementCorotationalTet* tet,
				  const SurgSim::Math::OdeState& state0,
				  const SurgSim::Math::RigidTransform3d& t,
				  double scale)
{
	SurgSim::Math::OdeState state;

	defineCurrentState(state0, &state, t, false);

	Matrix expectedK = Matrix::Zero(state.getNumDof(), state.getNumDof());
	SurgSim::Math::addSubMatrix(scale * tet->getRotatedStiffnessMatrix(state), tet->getNodeIds(), 3, &expectedK);

	Matrix K = Matrix::Zero(state.getNumDof(), state.getNumDof());
	tet->addStiffness(state, &K, scale);

	EXPECT_TRUE(K.isApprox(expectedK));
}

void testAddMass(MockFem3DElementCorotationalTet* tet,
				 const SurgSim::Math::OdeState& state0,
				 const SurgSim::Math::RigidTransform3d& t,
				 double scale)
{
	SurgSim::Math::OdeState state;

	defineCurrentState(state0, &state, t, false);

	Eigen::Matrix<double, 12, 12> M0 = tet->getNonRotatedMassMatrix();
	Matrix33d R;
	Eigen::Matrix<double, 12, 12> Mrot;
	tet->computeRotationMassAndStiffness(state, &R, &Mrot, nullptr);
	Eigen::Matrix<double, 12, 12> R12x12 = make12x12(Eigen::Matrix<double, 3, 3>(R));

	Matrix expectedM = Matrix::Zero(state.getNumDof(), state.getNumDof());
	SurgSim::Math::addSubMatrix(scale * R12x12 * M0 * R12x12.transpose(), tet->getNodeIds(), 3, &expectedM);

	Matrix M = Matrix::Zero(state.getNumDof(), state.getNumDof());
	tet->addMass(state, &M, scale);

	EXPECT_TRUE(M.isApprox(expectedM));
}

void testAddFMDK(MockFem3DElementCorotationalTet* tet,
				 const SurgSim::Math::OdeState& state0,
				 const SurgSim::Math::RigidTransform3d& t)
{
	SurgSim::Math::OdeState state;

	defineCurrentState(state0, &state, t, false);

	Eigen::Matrix<double, 12, 12> K0 = tet->getNonRotatedStiffnessMatrix();
	Eigen::Matrix<double, 12, 12> M0 = tet->getNonRotatedMassMatrix();
	Matrix33d R;
	Eigen::Matrix<double, 12, 12> Mrot;
	tet->computeRotationMassAndStiffness(state, &R, &Mrot, nullptr);
	Eigen::Matrix<double, 12, 12> R12x12 = make12x12(Eigen::Matrix<double, 3, 3>(R));

	Vector expectedF = Vector::Zero(state.getNumDof());
	Matrix expectedM = Matrix::Zero(state.getNumDof(), state.getNumDof());
	Matrix expectedK = Matrix::Zero(state.getNumDof(), state.getNumDof());
	SurgSim::Math::addSubMatrix(tet->getRotatedStiffnessMatrix(state), tet->getNodeIds(), 3, &expectedK);
	SurgSim::Math::addSubMatrix(R12x12 * M0 * R12x12.transpose(), tet->getNodeIds(), 3, &expectedM);

	Eigen::Matrix<double, 12, 1> x;
	SurgSim::Math::getSubVector(state.getPositions(), tet->getNodeIds(), 3, &x);
	Eigen::Matrix<double, 12 ,1> f = - R12x12 * K0 * R12x12.transpose() * (x - (R12x12 * tet->getInitialPosition()));
	SurgSim::Math::addSubVector(f, tet->getNodeIds(), 3, &expectedF);

	Vector F = Vector::Zero(state.getNumDof());
	Matrix M = Matrix::Zero(state.getNumDof(), state.getNumDof());
	Matrix D = Matrix::Zero(state.getNumDof(), state.getNumDof());
	Matrix K = Matrix::Zero(state.getNumDof(), state.getNumDof());
	tet->addFMDK(state, &F, &M, &D, &K);

	EXPECT_TRUE(F.isApprox(expectedF));
	EXPECT_TRUE(M.isApprox(expectedM));
	EXPECT_TRUE(D.isZero());
	EXPECT_TRUE(K.isApprox(expectedK));
}
}; // anonymous namespace

TEST_F(Fem3DElementCorotationalTetrahedronTests, AddStiffnessTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::RigidTransform3d;

	MockFem3DElementCorotationalTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	{
		SCOPED_TRACE("Without rotation, scale 1.0");
		testAddStiffness(&tet, m_restState, RigidTransform3d::Identity(), 1.0);
	}

	{
		SCOPED_TRACE("Without rotation, scale 0.4");
		testAddStiffness(&tet, m_restState, RigidTransform3d::Identity(), 0.4);
	}

	{
		SCOPED_TRACE("With rotation, scale 1.0");
		testAddStiffness(&tet, m_restState, makeRigidTransform(m_rotation, Vector3d::Zero()), 1.0);
	}

	{
		SCOPED_TRACE("With rotation, scale 0.4");
		testAddStiffness(&tet, m_restState, makeRigidTransform(m_rotation, Vector3d::Zero()), 0.4);
	}
}

TEST_F(Fem3DElementCorotationalTetrahedronTests, AddMassTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::RigidTransform3d;

	MockFem3DElementCorotationalTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	{
		SCOPED_TRACE("Without rotation, scale 1.0");
		testAddMass(&tet, m_restState, RigidTransform3d::Identity(), 1.0);
	}

	{
		SCOPED_TRACE("Without rotation, scale 0.4");
		testAddMass(&tet, m_restState, RigidTransform3d::Identity(), 0.4);
	}

	{
		SCOPED_TRACE("With rotation, scale 1.0");
		testAddMass(&tet, m_restState, makeRigidTransform(m_rotation, Vector3d::Zero()), 1.0);
	}

	{
		SCOPED_TRACE("With rotation, scale 0.4");
		testAddMass(&tet, m_restState, makeRigidTransform(m_rotation, Vector3d::Zero()), 0.4);
	}
}

TEST_F(Fem3DElementCorotationalTetrahedronTests, AddFMDKTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::RigidTransform3d;

	MockFem3DElementCorotationalTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	{
		SCOPED_TRACE("Without rotation");
		testAddFMDK(&tet, m_restState, RigidTransform3d::Identity());
	}

	{
		SCOPED_TRACE("With rotation");
		testAddFMDK(&tet, m_restState, makeRigidTransform(m_rotation, Vector3d::Zero()));
	}
}

namespace
{
void testAddForce(MockFem3DElementCorotationalTet* tet,
				  const SurgSim::Math::OdeState& state0,
				  const SurgSim::Math::RigidTransform3d& t,
				  bool addLocalDeformation)
{
	SurgSim::Math::OdeState statet;
	SurgSim::Math::Matrix K = tet->getNonRotatedStiffnessMatrix();

	// F = -RK(R^t.x - x0)
	Eigen::Matrix<double, 12, 1> x, x0;
	SurgSim::Math::getSubVector(state0.getPositions(), tet->getNodeIds(), 3, &x0);
	defineCurrentState(state0, &statet, t, addLocalDeformation);
	SurgSim::Math::getSubVector(statet.getPositions(), tet->getNodeIds(), 3, &x);

	// Note that the element rotation is not necessarily the RigidTransform rotation
	// If addDeformation is true, it will add pseudo-random variation that will affect the rigid motion slightly
	SurgSim::Math::Matrix33d R = tet->getRotation(statet);
	Eigen::Matrix<double, 12, 12> R12x12 = Eigen::Matrix<double, 12, 12>::Zero();
	for (size_t nodeId = 0; nodeId < 4; ++nodeId)
	{
		R12x12.block<3, 3>(3 * nodeId, 3 * nodeId) = R;
	}

	SurgSim::Math::Vector expectedF;
	expectedF.setZero(statet.getNumDof());
	Eigen::Matrix<double, 12 ,1> f = - R12x12 * K * R12x12.transpose() * (x - (R12x12 * x0));
	SurgSim::Math::addSubVector(f, tet->getNodeIds(), 3, &expectedF);

	EXPECT_TRUE(tet->getRotation(statet).isApprox(R));
	EXPECT_TRUE(tet->getNonRotatedStiffnessMatrix().isApprox(K));
	EXPECT_TRUE(tet->getInitialPosition().isApprox(x0));
	{
		SCOPED_TRACE("Scale 1.0");
		SurgSim::Math::Vector F;
		F.setZero(statet.getNumDof());
		tet->addForce(statet, &F);
		EXPECT_LT((F - expectedF).norm(), epsilonAddForce);
		if (!addLocalDeformation)
		{
			EXPECT_TRUE(F.isZero(epsilonAddForce));
		}
	}

	{
		SCOPED_TRACE("Scale 0.4");
		SurgSim::Math::Vector F;
		F.setZero(statet.getNumDof());
		tet->addForce(statet, &F, 0.4);
		EXPECT_LT((F - 0.4 * expectedF).norm(), epsilonAddForce);
		if (!addLocalDeformation)
		{
			EXPECT_TRUE(F.isZero(epsilonAddForce));
		}
	}
}
}; // anonymous namespace

TEST_F(Fem3DElementCorotationalTetrahedronTests, AddForceTest)
{
	MockFem3DElementCorotationalTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	SurgSim::Math::RigidTransform3d transformation;
	transformation.linear() = m_rotation;
	transformation.translation() = m_translation;

	{
		SCOPED_TRACE("No deformation, no rigid transformation");
		testAddForce(&tet, m_restState, SurgSim::Math::RigidTransform3d::Identity(), false);
	}

	{
		SCOPED_TRACE("No deformation, rigid transformation");
		testAddForce(&tet, m_restState, transformation, false);
	}

	{
		SCOPED_TRACE("Deformation, no rigid transformation");
		testAddForce(&tet, m_restState, SurgSim::Math::RigidTransform3d::Identity(), true);
	}

	{
		SCOPED_TRACE("Deformation, rigid transformation");
		testAddForce(&tet, m_restState, transformation, true);
	}
}

TEST_F(Fem3DElementCorotationalTetrahedronTests, AddMatVecTest)
{
	MockFem3DElementCorotationalTet tet(m_nodeIds);
	tet.setupInitialParams(m_restState, m_rho, m_nu, m_E);

	SurgSim::Math::RigidTransform3d transformation;
	transformation.linear() = m_rotation;
	transformation.translation() = m_translation;

	SurgSim::Math::OdeState state;
	defineCurrentState(m_restState, &state, transformation, true);

	Eigen::Matrix<double, 12, 12> M = tet.getRotatedMassMatrix(state);
	Eigen::Matrix<double, 12, 12> K = tet.getRotatedStiffnessMatrix(state);

	SurgSim::Math::Vector ones = SurgSim::Math::Vector::Ones(state.getNumDof());

	{
		SCOPED_TRACE("Mass only");

		SurgSim::Math::Vector result = SurgSim::Math::Vector::Zero(state.getNumDof());
		tet.addMatVec(state, 1.4, 0.0, 0.0, ones, &result);

		SurgSim::Math::Vector expectedResult = SurgSim::Math::Vector::Zero(state.getNumDof());
		Eigen::Matrix<double, 12, 1> f = 1.4 * M * SurgSim::Math::Vector::Ones(12);
		SurgSim::Math::addSubVector(f, m_nodeIdsAsVector, 3, &expectedResult);

		EXPECT_TRUE(result.isApprox(expectedResult));
	}

	{
		SCOPED_TRACE("Damping only");

		SurgSim::Math::Vector result = SurgSim::Math::Vector::Zero(state.getNumDof());
		tet.addMatVec(state, 0.0, 1.5, 0.0, ones, &result);

		EXPECT_TRUE(result.isZero());
	}

	{
		SCOPED_TRACE("Stiffness only");

		SurgSim::Math::Vector result = SurgSim::Math::Vector::Zero(state.getNumDof());
		tet.addMatVec(state, 0.0, 0.0, 1.6, ones, &result);

		SurgSim::Math::Vector expectedResult = SurgSim::Math::Vector::Zero(state.getNumDof());
		Eigen::Matrix<double, 12, 1> f = 1.6 * K * SurgSim::Math::Vector::Ones(12);
		SurgSim::Math::addSubVector(f, m_nodeIdsAsVector, 3, &expectedResult);

		EXPECT_TRUE(result.isApprox(expectedResult));
	}

	{
		SCOPED_TRACE("Mass/Damping/Stiffness");

		SurgSim::Math::Vector result = SurgSim::Math::Vector::Zero(state.getNumDof());
		tet.addMatVec(state, 1.4, 1.5, 1.6, ones, &result);

		SurgSim::Math::Vector expectedResult = SurgSim::Math::Vector::Zero(state.getNumDof());
		Eigen::Matrix<double, 12, 1> f = (1.4 * M + 1.6 * K) * SurgSim::Math::Vector::Ones(12);
		SurgSim::Math::addSubVector(f, m_nodeIdsAsVector, 3, &expectedResult);

		EXPECT_TRUE(result.isApprox(expectedResult, epsilonAddMatVec));
	}
}
