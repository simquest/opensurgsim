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

#include <SurgSim/Physics/RigidActorParameters.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Valid.h>

namespace SurgSim
{

namespace Physics
{

class RigidActorParametersTest : public ::testing::Test
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

TEST_F(RigidActorParametersTest, ConstructorTest)
{
	ASSERT_NO_THROW( {RigidActorParameters rigidActorParam;});
}

TEST_F(RigidActorParametersTest, DefaultValueTest)
{
	// Create the base rigid actor state
	std::shared_ptr<RigidActorParameters> rigidActorParam = std::make_shared<RigidActorParameters>();

	// Mass density [default = 0]
	EXPECT_EQ(0.0, rigidActorParam->getDensity());
	// Mass [default = qNaA]
	EXPECT_FALSE(SurgSim::Math::isValid(rigidActorParam->getMass()));
	// Inertia 3x3 symmetric matrix [default = qNaN values]
	EXPECT_FALSE(SurgSim::Math::isValid(rigidActorParam->getLocalInertia()));
	// Linear damping [default = 0]
	EXPECT_EQ(0.0, rigidActorParam->getLinearDamping());
	// Angular damping [default = 0]
	EXPECT_EQ(0.0, rigidActorParam->getAngularDamping());
	// Mesh [default = nullptr]
	EXPECT_EQ(0u, rigidActorParam->getShapes().size());
	// RigidShape [default = nullptr]
	EXPECT_EQ(nullptr, rigidActorParam->getShapeUsedForMassInertia());
	// isValid [default = false]
	EXPECT_FALSE(rigidActorParam->isValid());
}

TEST_F(RigidActorParametersTest, SetGetTest)
{
	// Create the base rigid actor state
	std::shared_ptr<RigidActorParameters> rigidActorParam = std::make_shared<RigidActorParameters>();

	// isValid
	EXPECT_FALSE(rigidActorParam->isValid());

	// Mass density
	rigidActorParam->setDensity(m_density);
	EXPECT_EQ(m_density, rigidActorParam->getDensity());
	rigidActorParam->setDensity(0.0);
	EXPECT_EQ(0.0, rigidActorParam->getDensity());

	// Mass
	rigidActorParam->setMass(m_mass);
	EXPECT_EQ(m_mass, rigidActorParam->getMass());
	rigidActorParam->setMass(0.0);
	EXPECT_EQ(0.0, rigidActorParam->getMass());

	// isValid
	rigidActorParam->setMass(m_mass);
	EXPECT_FALSE(rigidActorParam->isValid());
	rigidActorParam->setMass(0.0);
	rigidActorParam->setLocalInertia(m_inertia);
	EXPECT_FALSE(rigidActorParam->isValid());
	rigidActorParam->setMass(m_mass);
	EXPECT_TRUE(rigidActorParam->isValid());

	// Inertia 3x3 symmetric matrix
	rigidActorParam->setLocalInertia(m_inertia);
	EXPECT_EQ(m_inertia, rigidActorParam->getLocalInertia());
	rigidActorParam->setLocalInertia(m_id33);
	EXPECT_EQ(m_id33, rigidActorParam->getLocalInertia());

	// Linear damping
	rigidActorParam->setLinearDamping(5.5);
	EXPECT_EQ(5.5, rigidActorParam->getLinearDamping());
	rigidActorParam->setLinearDamping(0.0);
	EXPECT_EQ(0.0, rigidActorParam->getLinearDamping());

	// Angular damping
	rigidActorParam->setAngularDamping(5.5);
	EXPECT_EQ(5.5, rigidActorParam->getAngularDamping());
	rigidActorParam->setAngularDamping(0.0);
	EXPECT_EQ(0.0, rigidActorParam->getAngularDamping());

	// RigidShape
	rigidActorParam->setShapeUsedForMassInertia(m_sphere);
	EXPECT_EQ(m_sphere, rigidActorParam->getShapeUsedForMassInertia());
	EXPECT_EQ(m_sphere, rigidActorParam->getShapes()[0]);
	rigidActorParam->setShapeUsedForMassInertia(nullptr);
	EXPECT_EQ(nullptr, rigidActorParam->getShapeUsedForMassInertia());
	EXPECT_EQ(1u, rigidActorParam->getShapes().size());

	// isValid
	rigidActorParam->setMass(m_mass);
	rigidActorParam->setLocalInertia(m_inertia);
	EXPECT_TRUE(rigidActorParam->isValid());
}

TEST_F(RigidActorParametersTest, ShapesTest)
{
	// Create the base rigid actor state
	std::shared_ptr<RigidActorParameters> rigidActorParam = std::make_shared<RigidActorParameters>();

	// RigidShape
	rigidActorParam->setShapeUsedForMassInertia(m_sphere);
	EXPECT_EQ(1u, rigidActorParam->getShapes().size());
	rigidActorParam->addShape(m_sphere);
	EXPECT_EQ(2u, rigidActorParam->getShapes().size());
	rigidActorParam->setShapeUsedForMassInertia(nullptr);
	EXPECT_EQ(2u, rigidActorParam->getShapes().size());
	rigidActorParam->removeShape(m_sphere);
	EXPECT_EQ(1u, rigidActorParam->getShapes().size());
	rigidActorParam->removeShape(m_sphere);
	EXPECT_EQ(0u, rigidActorParam->getShapes().size());
}

TEST_F(RigidActorParametersTest, ValidityCheckTest)
{
	double qNaN = std::numeric_limits<double>::quiet_NaN(); 

	// Create the base rigid actor state
	std::shared_ptr<RigidActorParameters> rigidActorParam = std::make_shared<RigidActorParameters>();

	// Mass invalid (qNaN), inertia invalid (qNaN)
	EXPECT_FALSE(rigidActorParam->isValid());

	// Mass valid, inertia invalid (qNaN)
	rigidActorParam->setMass(m_mass);
	EXPECT_FALSE(rigidActorParam->isValid());

	// Mass invalid (qNaN), inertia valid
	rigidActorParam->setMass(qNaN);
	rigidActorParam->setLocalInertia(m_inertia);
	EXPECT_FALSE(rigidActorParam->isValid());

	// Mass invalid (0), inertia valid
	rigidActorParam->setMass(0.0);
	rigidActorParam->setLocalInertia(m_inertia);
	EXPECT_FALSE(rigidActorParam->isValid());

	// Mass valid, inertia invalid (0)
	rigidActorParam->setMass(m_mass);
	rigidActorParam->setLocalInertia(m_zero33);
	EXPECT_FALSE(rigidActorParam->isValid());

	// Mass valid, inertia valid
	rigidActorParam->setMass(m_mass);
	rigidActorParam->setLocalInertia(m_inertia);
	EXPECT_TRUE(rigidActorParam->isValid());
}

TEST_F(RigidActorParametersTest, DensityWithSphereShapeTest)
{
	// Create the base rigid actor state
	std::shared_ptr<RigidActorParameters> rigidActorParam = std::make_shared<RigidActorParameters>();

	// Mass density
	rigidActorParam->setDensity(m_density);

	// Test isValid
	EXPECT_FALSE(rigidActorParam->isValid());

	// RigidShape
	rigidActorParam->setShapeUsedForMassInertia(m_sphere);

	// Test isValid
	EXPECT_TRUE(rigidActorParam->isValid());

	// Test inertia
	EXPECT_TRUE(rigidActorParam->getLocalInertia().isApprox(m_inertia));

	// Test mass
	EXPECT_EQ(m_mass, rigidActorParam->getMass());

	// Test mass center
	EXPECT_TRUE(rigidActorParam->getMassCenter().isZero());
}

}; // Physics

}; // SurgSim
