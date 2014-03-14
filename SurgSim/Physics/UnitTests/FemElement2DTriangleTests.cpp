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
#include "SurgSim/Physics/FemElement2DTriangle.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::FemElement2DTriangle;

namespace
{
const double epsilon = 1e-9;
};

class MockFemElement2D : public FemElement2DTriangle
{
public:
	MockFemElement2D(std::array<unsigned int, 3> nodeIds)
		: FemElement2DTriangle(nodeIds)
	{
	}

	const Eigen::Matrix<double, 18, 18, Eigen::DontAlign>& getInitialRotation() const
	{
		return m_R0;
	}

	double getRestArea() const
	{
		return m_restArea;
	}
};

class FemElement2DTriangleTests : public ::testing::Test
{
public:
	static const int m_numberNodes = 6;

	std::array<unsigned int, 3> m_nodeIds;
	DeformableRepresentationState m_restState;
	double m_expectedVolume;
	double m_rho, m_E, m_nu;
	double m_A;         // area
	double m_thickness; // thickness
	Quaterniond m_expectedRotation;

	virtual void SetUp() override
	{
		using SurgSim::Math::getSubVector;

		m_rho = 1000.0;
		m_E = 1e6;
		m_nu = 0.45;
		m_thickness = 1e-2;
		m_A = 1.0 / 2.0;
		m_expectedVolume = m_A * m_thickness;

		// Triangle is made of node 3, 1 and 5 in a bigger system containing m_numberNodes nodes (at least 6)
		m_nodeIds[0] = 3;
		m_nodeIds[1] = 1;
		m_nodeIds[2] = 5;

		m_restState.setNumDof(6, m_numberNodes);

		m_expectedRotation.coeffs().setRandom();
		m_expectedRotation.normalize();

		Vector3d A(0.0, 0.0, 0.0);
		Vector3d B(1.0, 0.0, 0.0);
		Vector3d C(0.0, 1.0, 0.0);
		// The initial rotation of ABC is defined by (i=AB, j=AC, k=ABxAC) = Identity
		// Therefore, by applying m_expectedRotation to the points, the initial rotation of the
		// element should be m_expectedRotation

		Vector& x = m_restState.getPositions();
		getSubVector(x, m_nodeIds[0], 6).segment<3>(0) = m_expectedRotation._transformVector(A);
		getSubVector(x, m_nodeIds[1], 6).segment<3>(0) = m_expectedRotation._transformVector(B);
		getSubVector(x, m_nodeIds[2], 6).segment<3>(0) = m_expectedRotation._transformVector(C);
	}

	void getExpectedMassMatrix(Eigen::Ref<SurgSim::Math::Matrix> mass)
	{
		// XXX TO DO
	}

	void getExpectedStiffnessMatrix(Eigen::Ref<SurgSim::Math::Matrix> stiffness)
	{
		// XXX TO DO
	}

	std::shared_ptr<MockFemElement2D> getElement()
	{
		auto element = std::make_shared<MockFemElement2D>(m_nodeIds);
		element->setThickness(m_thickness);
		element->setMassDensity(m_rho);
		element->setPoissonRatio(m_nu);
		element->setYoungModulus(m_E);
		element->initialize(m_restState);
		return element;
	}
};

TEST_F(FemElement2DTriangleTests, ConstructorTest)
{
	ASSERT_NO_THROW({ MockFemElement2D triangle(m_nodeIds); });
}

TEST_F(FemElement2DTriangleTests, NodeIdsTest)
{
	FemElement2DTriangle element(m_nodeIds);
	EXPECT_EQ(3u, element.getNumNodes());
	EXPECT_EQ(3u, element.getNodeIds().size());
	for (int i = 0; i < 3; i++)
	{
		EXPECT_EQ(m_nodeIds[i], element.getNodeId(i));
		EXPECT_EQ(m_nodeIds[i], element.getNodeIds()[i]);
	}
}

TEST_F(FemElement2DTriangleTests, setGetThicknessTest)
{
	FemElement2DTriangle element(m_nodeIds);

	// Default thickness = 0.0
	EXPECT_DOUBLE_EQ(0.0, element.getThickness());
	// Set to a valid thickness
	element.setThickness(1.54);
	EXPECT_DOUBLE_EQ(1.54, element.getThickness());
	// Set to an invalid thickness
	EXPECT_ANY_THROW(element.setThickness(0.0));
	EXPECT_ANY_THROW(element.setThickness(-9.4));
}

TEST_F(FemElement2DTriangleTests, MaterialParameterTest)
{
	FemElement2DTriangle element(m_nodeIds);
	element.setThickness(m_thickness);

	// Test the various mode of failure related to the physical parameters
	// This has been already tested in FemElementTests, but this is to make sure this method is called properly
	// So the same behavior should be expected
	{
		// Mass density not set
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Poisson Ratio not set
		element.setMassDensity(-1234.56);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Young modulus not set
		element.setPoissonRatio(0.55);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Invalid mass density
		element.setYoungModulus(-4321.33);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Invalid Poisson ratio
		element.setMassDensity(m_rho);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		// Invalid Young modulus
		element.setPoissonRatio(m_nu);
		ASSERT_ANY_THROW(element.initialize(m_restState));

		element.setYoungModulus(m_E);
		ASSERT_NO_THROW(element.initialize(m_restState));
	}
}

TEST_F(FemElement2DTriangleTests, VolumeTest)
{
	std::shared_ptr<MockFemElement2D> element = getElement();
	EXPECT_NEAR(element->getVolume(m_restState), m_expectedVolume, 1e-10);
}

TEST_F(FemElement2DTriangleTests, RestAreaTest)
{
	std::shared_ptr<MockFemElement2D> element = getElement();
	EXPECT_NEAR(element->getRestArea(), m_A, 1e-10);
}

TEST_F(FemElement2DTriangleTests, InitialRotationTest)
{
	std::shared_ptr<MockFemElement2D> element = getElement();

	// Use a mask to test the structure of the rotation matrix R0 (6 digonal block 3x3 matrix and 0 elsewhere)
	Eigen::Matrix<double, 18, 18> mask;
	mask.setOnes();
	mask.block<3, 3>(0, 0).setZero();
	mask.block<3, 3>(3, 3).setZero();
	mask.block<3, 3>(6, 6).setZero();
	mask.block<3, 3>(9, 9).setZero();
	mask.block<3, 3>(12, 12).setZero();
	mask.block<3, 3>(15, 15).setZero();
	EXPECT_TRUE(element->getInitialRotation().cwiseProduct(mask).isZero());

	EXPECT_TRUE(element->getInitialRotation().block(0, 0, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotation().block(3, 3, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotation().block(6, 6, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotation().block(9, 9, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotation().block(12, 12, 3, 3).isApprox(m_expectedRotation.matrix()));
	EXPECT_TRUE(element->getInitialRotation().block(15, 15, 3, 3).isApprox(m_expectedRotation.matrix()));
}

TEST_F(FemElement2DTriangleTests, ForceAndMatricesTest)
{
	// XXX TO DO
}
