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

/// \file Fem3DRepresentationTests.cpp
/// This file tests the non-abstract functionalities of the base class FemRepresentation

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h" //< Used to initialize the Component Fem3DRepresentation

#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector;
using SurgSim::Math::getSubVector;
using SurgSim::Math::Matrix;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{

namespace Physics
{

class Fem3DRepresentationTests : public ::testing::Test
{
public:
	double m_rho;
	double m_nu;
	double m_E;
	double m_dt;
	RigidTransform3d m_initialPose;
	std::shared_ptr<DeformableRepresentationState> m_initialState;
	Vector m_initialPositions, m_initialVelocities, m_initialAccelerations;
	Vector m_expectedTransformedPositions, m_expectedTransformedVelocities, m_expectedTransformedAccelerations;
	std::shared_ptr<Fem3DRepresentation> m_fem;
	std::array<unsigned int, 4> m_femElementNodeIdx;
	double m_expectedVolume;

	SurgSim::Math::Matrix m_expectedMassMatrix, m_expectedDampingMatrix, m_expectedStiffnessMatrix;
	SurgSim::Math::Vector m_expectedF;

protected:
	virtual void SetUp() override
	{
		m_rho = 2000.0;
		m_nu = 0.45;
		m_E = 1e6;
		m_dt = 1e-3;

		m_femElementNodeIdx[0] = 0;
		m_femElementNodeIdx[1] = 1;
		m_femElementNodeIdx[2] = 2;
		m_femElementNodeIdx[3] = 3;

		Quaterniond q(1.9, 4.2, 9.3, 2.1);
		q.normalize();
		Vector3d t(0.1, 0.2, 0.3);
		m_initialPose = SurgSim::Math::makeRigidTransform(q, t);

		m_fem = std::make_shared<Fem3DRepresentation>("name");

		m_initialState = std::make_shared<DeformableRepresentationState>();
		m_initialState->setNumDof(m_fem->getNumDofPerNode(), 4);

		m_initialPositions.resize(m_initialState->getNumDof());
		m_initialVelocities.resize(m_initialState->getNumDof());
		m_initialAccelerations.resize(m_initialState->getNumDof());
		m_expectedTransformedPositions.resize(m_initialState->getNumDof());
		m_expectedTransformedVelocities.resize(m_initialState->getNumDof());
		m_expectedTransformedAccelerations.resize(m_initialState->getNumDof());

		Vector& x = m_initialState->getPositions();
		getSubVector(m_initialPositions, 0, 3) = Vector3d(0.0, 0.0, 0.0);
		getSubVector(m_initialPositions, 1, 3) = Vector3d(1.0, 0.0, 0.0);
		getSubVector(m_initialPositions, 2, 3) = Vector3d(0.0, 1.0, 0.0);
		getSubVector(m_initialPositions, 3, 3) = Vector3d(0.0, 0.0, 1.0);
		getSubVector(x, 0, 3) = getSubVector(m_initialPositions, 0, 3);
		getSubVector(x, 1, 3) = getSubVector(m_initialPositions, 1, 3);
		getSubVector(x, 2, 3) = getSubVector(m_initialPositions, 2, 3);
		getSubVector(x, 3, 3) = getSubVector(m_initialPositions, 3, 3);
		getSubVector(m_expectedTransformedPositions, 0, 3) = m_initialPose * Vector3d(getSubVector(x, 0, 3));
		getSubVector(m_expectedTransformedPositions, 1, 3) = m_initialPose * Vector3d(getSubVector(x, 1, 3));
		getSubVector(m_expectedTransformedPositions, 2, 3) = m_initialPose * Vector3d(getSubVector(x, 2, 3));
		getSubVector(m_expectedTransformedPositions, 3, 3) = m_initialPose * Vector3d(getSubVector(x, 3, 3));

		Vector& v = m_initialState->getVelocities();
		getSubVector(m_initialVelocities, 0, 3) = Vector3d(0.0, 0.0, 0.0);
		getSubVector(m_initialVelocities, 1, 3) = Vector3d(2.0, 0.0, 0.0);
		getSubVector(m_initialVelocities, 2, 3) = Vector3d(0.0, 2.0, 0.0);
		getSubVector(m_initialVelocities, 3, 3) = Vector3d(0.0, 0.0, 2.0);
		getSubVector(v, 0, 3) = getSubVector(m_initialVelocities, 0, 3);
		getSubVector(v, 1, 3) = getSubVector(m_initialVelocities, 1, 3);
		getSubVector(v, 2, 3) = getSubVector(m_initialVelocities, 2, 3);
		getSubVector(v, 3, 3) = getSubVector(m_initialVelocities, 3, 3);
		getSubVector(m_expectedTransformedVelocities, 0, 3) = m_initialPose.linear() * getSubVector(v, 0, 3);
		getSubVector(m_expectedTransformedVelocities, 1, 3) = m_initialPose.linear() * getSubVector(v, 1, 3);
		getSubVector(m_expectedTransformedVelocities, 2, 3) = m_initialPose.linear() * getSubVector(v, 2, 3);
		getSubVector(m_expectedTransformedVelocities, 3, 3) = m_initialPose.linear() * getSubVector(v, 3, 3);

		Vector& a = m_initialState->getAccelerations();
		getSubVector(m_initialAccelerations, 0, 3) = Vector3d(0.0, 0.0, 0.0);
		getSubVector(m_initialAccelerations, 1, 3) = Vector3d(3.0, 0.0, 0.0);
		getSubVector(m_initialAccelerations, 2, 3) = Vector3d(0.0, 3.0, 0.0);
		getSubVector(m_initialAccelerations, 3, 3) = Vector3d(0.0, 0.0, 3.0);
		getSubVector(a, 0, 3) = getSubVector(m_initialAccelerations, 0, 3);
		getSubVector(a, 1, 3) = getSubVector(m_initialAccelerations, 1, 3);
		getSubVector(a, 2, 3) = getSubVector(m_initialAccelerations, 2, 3);
		getSubVector(a, 3, 3) = getSubVector(m_initialAccelerations, 3, 3);
		getSubVector(m_expectedTransformedAccelerations, 0, 3) = m_initialPose.linear() * getSubVector(a, 0, 3);
		getSubVector(m_expectedTransformedAccelerations, 1, 3) = m_initialPose.linear() * getSubVector(a, 1, 3);
		getSubVector(m_expectedTransformedAccelerations, 2, 3) = m_initialPose.linear() * getSubVector(a, 2, 3);
		getSubVector(m_expectedTransformedAccelerations, 3, 3) = m_initialPose.linear() * getSubVector(a, 3, 3);

		std::shared_ptr<FemElement3DTetrahedron> element;
		element = std::make_shared<FemElement3DTetrahedron>(m_femElementNodeIdx);
		element->setMassDensity(m_rho);
		element->setYoungModulus(m_E);
		element->setPoissonRatio(m_nu);
		m_fem->addFemElement(element);

		m_expectedVolume = 1.0 / 6.0;

		computeExpectedFMDK();
	}

	void computeExpectedFMDK()
	{

		m_expectedMassMatrix.resize(3*4, 3*4);
		m_expectedMassMatrix.setZero();
		m_expectedDampingMatrix.resize(3*4, 3*4);
		m_expectedDampingMatrix.setZero();
		m_expectedStiffnessMatrix.resize(3*4, 3*4);
		m_expectedStiffnessMatrix.setZero();
		m_expectedF.resize(3*4);
		m_expectedF.setZero();

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
			SurgSim::Math::Matrix& K = m_expectedStiffnessMatrix;

			// Expecte stiffness matrix given for our case in:
			// http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch09.d/AFEM.Ch09.pdf
			double E = m_E / (12.0*(1.0 - 2.0*m_nu)*(1.0 + m_nu));
			double n0 = 1.0 - 2.0 * m_nu;
			double n1 = 1.0 - m_nu;

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

		// No gravity, no rayleigh damping and using the initial state => no forces should be generated
		m_expectedF.setZero();
	}
};

TEST_F(Fem3DRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW({Fem3DRepresentation ("name");});
	ASSERT_NO_THROW({Fem3DRepresentation* fem = new Fem3DRepresentation("name"); delete fem;});
	ASSERT_NO_THROW({std::shared_ptr<Fem3DRepresentation> fem = std::make_shared<Fem3DRepresentation>("name");});
}

TEST_F(Fem3DRepresentationTests, GetTypeTest)
{
	EXPECT_EQ(REPRESENTATION_TYPE_FEM3D , m_fem->getType());
}

TEST_F(Fem3DRepresentationTests, TransformInitialStateTest)
{
	m_fem->setInitialPose(m_initialPose);
	m_fem->setInitialState(m_initialState);

	EXPECT_TRUE(m_fem->getInitialState()->getPositions().isApprox(m_expectedTransformedPositions));
	EXPECT_TRUE(m_fem->getInitialState()->getVelocities().isApprox(m_expectedTransformedVelocities));
	EXPECT_TRUE(m_fem->getInitialState()->getAccelerations().isApprox(m_expectedTransformedAccelerations));
}


TEST_F(Fem3DRepresentationTests, UpdateTest)
{
	using SurgSim::Framework::Runtime;

	// Need to call beforeUpdate() prior to calling update()
	// + Need to call setInitialState() prior to calling beforeUpdate()
	ASSERT_ANY_THROW(m_fem->update(m_dt));

	m_fem->setInitialState(m_initialState);
	m_fem->beforeUpdate(m_dt);
	// Need to call Initialize after addFemElement and setInitialState to initialize the mass information
	ASSERT_ANY_THROW(m_fem->update(m_dt));

	ASSERT_TRUE(m_fem->initialize(std::make_shared<Runtime>()));
	ASSERT_NO_THROW(m_fem->update(m_dt));

	// Previous and current state should contains the proper information
	// Note that the default integration scheme is Explicit Euler: x(t+dt) = x(t) + dt. v(t)
	// Note that the previous state should be the initial state, but current state should be different
	Vector expectCurrentPositions = m_fem->getPreviousState()->getPositions();
	expectCurrentPositions += m_dt * m_fem->getPreviousState()->getVelocities();
	EXPECT_TRUE(*m_fem->getPreviousState() == *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() != *m_fem->getInitialState());
	EXPECT_TRUE(m_fem->getCurrentState()->getPositions().isApprox(expectCurrentPositions));
}

TEST_F(Fem3DRepresentationTests, AfterUpdateTest)
{
	using SurgSim::Framework::Runtime;

	// Need to call setInitialState() prior to calling afterUpdate()
	ASSERT_ANY_THROW(m_fem->afterUpdate(m_dt));

	m_fem->setInitialState(m_initialState);
	ASSERT_TRUE(m_fem->initialize(std::make_shared<Runtime>()));
	m_fem->beforeUpdate(m_dt);
	m_fem->update(m_dt);
	ASSERT_NO_THROW(m_fem->afterUpdate(m_dt));

	// Final and current state should contains the same information
	// Note that the previous state should be the initial state, but current state should be different
	EXPECT_TRUE(*m_fem->getPreviousState() == *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() != *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() == *m_fem->getFinalState());
}

TEST_F(Fem3DRepresentationTests, ApplyCorrectionTest)
{
	double epsilon = 1e-12;
	m_fem->setInitialState(m_initialState);

	SurgSim::Math::Vector dv;
	dv.resize(m_fem->getNumDof());
	for (unsigned int i = 0; i < m_fem->getNumDof(); i++)
	{
		dv(i) = static_cast<double>(i);
	}

	ASSERT_LE(3u, m_fem->getNumDof());
	ASSERT_NEAR(2.0, dv(2), epsilon);

	Eigen::VectorXd previousX = m_fem->getCurrentState()->getPositions();
	Eigen::VectorXd previousV = m_fem->getCurrentState()->getVelocities();

	m_fem->applyCorrection(m_dt, dv.segment(0, m_fem->getNumDof()));
	Eigen::VectorXd nextX = m_fem->getCurrentState()->getPositions();
	Eigen::VectorXd nextV = m_fem->getCurrentState()->getVelocities();

	EXPECT_TRUE(nextX.isApprox(previousX + dv * m_dt, epsilon));
	EXPECT_TRUE(nextV.isApprox(previousV + dv, epsilon));

	dv(0) = std::numeric_limits<double>::infinity();
	EXPECT_TRUE(m_fem->isActive());
	m_fem->applyCorrection(m_dt, dv.segment(0, m_fem->getNumDof()));
	EXPECT_FALSE(m_fem->isActive());
}

namespace
{
void testMatrix(Matrix M, Matrix expectedM)
{
	if (expectedM.isZero())
	{
		EXPECT_TRUE(M.isZero());
	}
	else
	{
		EXPECT_TRUE(M.isApprox(expectedM));
	}
}
void testVector(Vector V, Vector expectedV)
{
	if (expectedV.isZero())
	{
		EXPECT_TRUE(V.isZero());
	}
	else
	{
		EXPECT_TRUE(V.isApprox(expectedV));
	}
}
}

TEST_F(Fem3DRepresentationTests, ComputesTest)
{
	using SurgSim::Framework::Runtime;

	m_fem->setInitialState(m_initialState);
	ASSERT_TRUE(m_fem->initialize(std::make_shared<Runtime>()));
	m_fem->setIsGravityEnabled(false);

	testMatrix(m_fem->computeM(*m_initialState), m_expectedMassMatrix);
	testMatrix(m_fem->computeD(*m_initialState), m_expectedDampingMatrix);
	testMatrix(m_fem->computeK(*m_initialState), m_expectedStiffnessMatrix);
	testVector(m_fem->computeF(*m_initialState), m_expectedF);

	Vector *f;
	Matrix *M, *D, *K;
	m_fem->computeFMDK(*m_initialState, &f, &M, &D, &K);
	testMatrix(*M, m_expectedMassMatrix);
	testMatrix(*D, m_expectedDampingMatrix);
	testMatrix(*K, m_expectedStiffnessMatrix);
	testVector(*f, m_expectedF);
}

TEST_F(Fem3DRepresentationTests, SetFilenameTest)
{
	{
		SCOPED_TRACE("Calling setFileName with real file");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		ASSERT_NO_THROW(fem->setFilename("Data/PlyReaderTests/Tetrahedron.ply"));
		ASSERT_TRUE(fem->loadFile());

		EXPECT_EQ(3u, fem->getNumDofPerNode());
		EXPECT_EQ(3u * 26u, fem->getNumDof());
		EXPECT_EQ(24u, fem->getInitialState()->getNumBoundaryConditions());
	}

	{
		SCOPED_TRACE("Calling setFileName with bad name");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		ASSERT_NO_THROW(fem->setFilename("Non existent fake name"));
		EXPECT_FALSE(fem->loadFile());
	}

	{
		SCOPED_TRACE("Loading file with no filename");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		EXPECT_FALSE(fem->loadFile());
	}

	{
		SCOPED_TRACE("Loading twice");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		ASSERT_NO_THROW(fem->setFilename("Data/PlyReaderTests/Tetrahedron.ply"));
		ASSERT_TRUE(fem->loadFile());
		ASSERT_FALSE(fem->loadFile());
	}

	{
		SCOPED_TRACE("Loading with non-shared ptr");
		Fem3DRepresentation fem("fem3d");

		ASSERT_NO_THROW(fem.setFilename("Data/PlyReaderTests/Tetrahedron.ply"));
		EXPECT_THROW(fem.loadFile(), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Loading file with incorrect PLY format");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		ASSERT_NO_THROW(fem->setFilename("Data/PlyReaderTests/WrongPlyTetrahedron.ply"));
		EXPECT_FALSE(fem->loadFile());
	}

	{
		SCOPED_TRACE("Loading file with incorrect data");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		ASSERT_NO_THROW(fem->setFilename("Data/PlyReaderTests/WrongDataTetrahedron.ply"));
		EXPECT_THROW(fem->loadFile(), SurgSim::Framework::AssertionFailure);
	}
}

} // namespace Physics

} // namespace SurgSim
