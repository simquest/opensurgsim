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

#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::SphereShape;

namespace SurgSim
{

namespace Physics
{

class RigidRepresentationParametersTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_radius = 1.0;
		m_density = 9000.0;
		m_mass = 4.0 / 3.0 * M_PI * m_radius * m_radius * m_radius * m_density;
		double coef = 2.0 / 5.0 * m_mass * m_radius * m_radius;
		m_inertia << coef, 0.0, 0.0, 0.0, coef, 0.0, 0.0, 0.0, coef;
		m_id33.setIdentity();
		m_zero33.setZero();
		m_sphere = std::make_shared<SphereShape>(m_radius);
	}

	void TearDown()
	{
	}

	// Sphere radius (in m)
	double m_radius;

	// Sphere density (in Kg.m-3)
	double m_density;

	// Sphere mass (in Kg)
	double m_mass;

	// Sphere inertia matrix
	SurgSim::Math::Matrix33d m_inertia;

	// Identity matrix 3x3 (for convenience)
	SurgSim::Math::Matrix33d m_id33;

	// Zero matrix 3x3 (for convenience)
	SurgSim::Math::Matrix33d m_zero33;

	// SphereShape
	std::shared_ptr<SphereShape> m_sphere;
};

TEST_F(RigidRepresentationParametersTest, ConstructorTest)
{
	ASSERT_NO_THROW( {RigidRepresentationParameters rigidRepresentationParam;});
}

TEST_F(RigidRepresentationParametersTest, DefaultValueTest)
{
	// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationParameters> rigidRepresentationParam =
		std::make_shared<RigidRepresentationParameters>();

	// Mass density [default = 0]
	EXPECT_EQ(0.0, rigidRepresentationParam->getDensity());
	// Mass [default = qNaA]
	EXPECT_FALSE(SurgSim::Math::isValid(rigidRepresentationParam->getMass()));
	// Inertia 3x3 symmetric matrix [default = qNaN values]
	EXPECT_FALSE(SurgSim::Math::isValid(rigidRepresentationParam->getLocalInertia()));
	// Linear damping [default = 0]
	EXPECT_EQ(0.0, rigidRepresentationParam->getLinearDamping());
	// Angular damping [default = 0]
	EXPECT_EQ(0.0, rigidRepresentationParam->getAngularDamping());
	// Mesh [default = nullptr]
	EXPECT_EQ(0u, rigidRepresentationParam->getShapes().size());
	// Shape [default = nullptr]
	EXPECT_EQ(nullptr, rigidRepresentationParam->getShapeUsedForMassInertia());
	// isValid [default = false]
	EXPECT_FALSE(rigidRepresentationParam->isValid());
}

TEST_F(RigidRepresentationParametersTest, SetGetTest)
{
	// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationParameters> rigidRepresentationParam =
		std::make_shared<RigidRepresentationParameters>();

	// isValid
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Mass density
	rigidRepresentationParam->setDensity(m_density);
	EXPECT_EQ(m_density, rigidRepresentationParam->getDensity());
	rigidRepresentationParam->setDensity(0.0);
	EXPECT_EQ(0.0, rigidRepresentationParam->getDensity());

	// Mass
	rigidRepresentationParam->setMass(m_mass);
	EXPECT_EQ(m_mass, rigidRepresentationParam->getMass());
	rigidRepresentationParam->setMass(0.0);
	EXPECT_EQ(0.0, rigidRepresentationParam->getMass());

	// isValid
	rigidRepresentationParam->setMass(m_mass);
	EXPECT_FALSE(rigidRepresentationParam->isValid());
	rigidRepresentationParam->setMass(0.0);
	rigidRepresentationParam->setLocalInertia(m_inertia);
	EXPECT_FALSE(rigidRepresentationParam->isValid());
	rigidRepresentationParam->setMass(m_mass);
	EXPECT_TRUE(rigidRepresentationParam->isValid());

	// Inertia 3x3 symmetric matrix
	rigidRepresentationParam->setLocalInertia(m_inertia);
	EXPECT_EQ(m_inertia, rigidRepresentationParam->getLocalInertia());
	rigidRepresentationParam->setLocalInertia(m_id33);
	EXPECT_EQ(m_id33, rigidRepresentationParam->getLocalInertia());

	// Linear damping
	rigidRepresentationParam->setLinearDamping(5.5);
	EXPECT_EQ(5.5, rigidRepresentationParam->getLinearDamping());
	rigidRepresentationParam->setLinearDamping(0.0);
	EXPECT_EQ(0.0, rigidRepresentationParam->getLinearDamping());

	// Angular damping
	rigidRepresentationParam->setAngularDamping(5.5);
	EXPECT_EQ(5.5, rigidRepresentationParam->getAngularDamping());
	rigidRepresentationParam->setAngularDamping(0.0);
	EXPECT_EQ(0.0, rigidRepresentationParam->getAngularDamping());

	// Shape
	rigidRepresentationParam->setShapeUsedForMassInertia(m_sphere);
	EXPECT_EQ(m_sphere, rigidRepresentationParam->getShapeUsedForMassInertia());
	EXPECT_EQ(m_sphere, rigidRepresentationParam->getShapes()[0]);
	rigidRepresentationParam->setShapeUsedForMassInertia(nullptr);
	EXPECT_EQ(nullptr, rigidRepresentationParam->getShapeUsedForMassInertia());
	EXPECT_EQ(1u, rigidRepresentationParam->getShapes().size());

	// isValid
	rigidRepresentationParam->setMass(m_mass);
	rigidRepresentationParam->setLocalInertia(m_inertia);
	EXPECT_TRUE(rigidRepresentationParam->isValid());
}

TEST_F(RigidRepresentationParametersTest, ShapesTest)
{
	// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationParameters> rigidRepresentationParam =
		std::make_shared<RigidRepresentationParameters>();

	// Shape
	rigidRepresentationParam->setShapeUsedForMassInertia(m_sphere);
	EXPECT_EQ(1u, rigidRepresentationParam->getShapes().size());
	rigidRepresentationParam->addShape(m_sphere);
	EXPECT_EQ(2u, rigidRepresentationParam->getShapes().size());
	rigidRepresentationParam->setShapeUsedForMassInertia(nullptr);
	EXPECT_EQ(2u, rigidRepresentationParam->getShapes().size());
	rigidRepresentationParam->removeShape(m_sphere);
	EXPECT_EQ(1u, rigidRepresentationParam->getShapes().size());
	rigidRepresentationParam->removeShape(m_sphere);
	EXPECT_EQ(0u, rigidRepresentationParam->getShapes().size());
}

TEST_F(RigidRepresentationParametersTest, ValidityCheckTest)
{
	double qNaN = std::numeric_limits<double>::quiet_NaN();

	// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationParameters> rigidRepresentationParam =
		std::make_shared<RigidRepresentationParameters>();

	// Mass invalid (qNaN), inertia invalid (qNaN)
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Mass valid, inertia invalid (qNaN)
	rigidRepresentationParam->setMass(m_mass);
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Mass invalid (qNaN), inertia valid
	rigidRepresentationParam->setMass(qNaN);
	rigidRepresentationParam->setLocalInertia(m_inertia);
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Mass invalid (0), inertia valid
	rigidRepresentationParam->setMass(0.0);
	rigidRepresentationParam->setLocalInertia(m_inertia);
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Mass valid, inertia invalid (0)
	rigidRepresentationParam->setMass(m_mass);
	rigidRepresentationParam->setLocalInertia(m_zero33);
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Mass valid, inertia valid
	rigidRepresentationParam->setMass(m_mass);
	rigidRepresentationParam->setLocalInertia(m_inertia);
	EXPECT_TRUE(rigidRepresentationParam->isValid());
}

TEST_F(RigidRepresentationParametersTest, DensityWithSphereShapeTest)
{
	// Create the base rigid representation state
	std::shared_ptr<RigidRepresentationParameters> rigidRepresentationParam =
		std::make_shared<RigidRepresentationParameters>();

	// Mass density
	rigidRepresentationParam->setDensity(m_density);

	// Test isValid
	EXPECT_FALSE(rigidRepresentationParam->isValid());

	// Shape
	rigidRepresentationParam->setShapeUsedForMassInertia(m_sphere);

	// Test isValid
	EXPECT_TRUE(rigidRepresentationParam->isValid());

	// Test inertia
	EXPECT_TRUE(rigidRepresentationParam->getLocalInertia().isApprox(m_inertia));

	// Test mass
	EXPECT_EQ(m_mass, rigidRepresentationParam->getMass());

	// Test mass center
	EXPECT_TRUE(rigidRepresentationParam->getMassCenter().isZero());
}

}; // Physics

}; // SurgSim
