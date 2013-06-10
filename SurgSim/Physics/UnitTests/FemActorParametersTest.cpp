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

#include <SurgSim/Physics/FemActorParameters.h>

namespace
{
	const double epsilon = 1e-10;
}

namespace SurgSim
{

namespace Physics
{

TEST(FemActorParametersTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FemActorParameters femActorParam;});
}

TEST(FemActorParametersTest, DefaultValueTest)
{
	// Create the base rigid actor state
	std::shared_ptr<FemActorParameters> femActorParam = std::make_shared<FemActorParameters>();

	// Mass density [default = 0]
	EXPECT_NEAR(0.0, femActorParam->getDensity(), epsilon);
	// Rayleigh damping mass parameter [default = 0]
	EXPECT_NEAR(0.0, femActorParam->getRayleighDampingMass(), epsilon);
	// Rayleigh damping stiffness parameter [default = 0]
	EXPECT_NEAR(0.0, femActorParam->getRayleighDampingStiffness(), epsilon);
	// Young modulus [default = 0]
	EXPECT_NEAR(0.0, femActorParam->getYoungModulus(), epsilon);
	// Poisson ratio [default = 0]
	EXPECT_NEAR(0.0, femActorParam->getPoissonRatio(), epsilon);
	// isValid [default = false]
	EXPECT_FALSE(femActorParam->isValid());
}

TEST(FemActorParametersTest, SetGetValidTest)
{
	// Create the base rigid actor state
	std::shared_ptr<FemActorParameters> femActorParam = std::make_shared<FemActorParameters>();

	// Set proper density, young modulus and poisson ratio to test validy flag
	femActorParam->setDensity(12.52);
	EXPECT_NEAR(12.52, femActorParam->getDensity(), epsilon);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setYoungModulus(10e7);
	EXPECT_NEAR(10e7, femActorParam->getYoungModulus(), epsilon);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(0.4);
	EXPECT_NEAR(0.4, femActorParam->getPoissonRatio(), epsilon);
	EXPECT_TRUE(femActorParam->isValid());

	// Test Rayleigh damping mass
	femActorParam->setRayleighDampingMass(13.21);
	EXPECT_NEAR(13.21, femActorParam->getRayleighDampingMass(), epsilon);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setRayleighDampingMass(-13.21);
	EXPECT_NEAR(-13.21, femActorParam->getRayleighDampingMass(), epsilon);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setRayleighDampingMass(0.0);
	EXPECT_NEAR(0.0, femActorParam->getRayleighDampingMass(), epsilon);
	EXPECT_TRUE(femActorParam->isValid());

	// Test Rayleigh damping stiffness
	femActorParam->setRayleighDampingStiffness(3.87);
	EXPECT_NEAR(3.87, femActorParam->getRayleighDampingStiffness(), epsilon);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setRayleighDampingStiffness(-3.87);
	EXPECT_NEAR(-3.87, femActorParam->getRayleighDampingStiffness(), epsilon);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setRayleighDampingStiffness(0.0);
	EXPECT_NEAR(0.0, femActorParam->getRayleighDampingStiffness(), epsilon);
	EXPECT_TRUE(femActorParam->isValid());

	// Test validity flag some more
	femActorParam->setDensity(0.0);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setDensity(1.0);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setYoungModulus(0.0);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setYoungModulus(1.0);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(-2.0);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setPoissonRatio(-1.0);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setPoissonRatio(-0.99);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(0.0);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(0.499);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(0.5);
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setPoissonRatio(1.0);
	EXPECT_FALSE(femActorParam->isValid());
}


TEST(FemActorParametersTest, BoundaryConditionsTest)
{
	// Create the base rigid actor state
	std::shared_ptr<FemActorParameters> femActorParam = std::make_shared<FemActorParameters>();

	// Add 1 by 1
	EXPECT_EQ(0u, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->addBoundaryCondition(0));
	EXPECT_EQ(1u, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->addBoundaryCondition(1));
	EXPECT_EQ(2u, femActorParam->getBoundaryConditions().size());
	EXPECT_FALSE(femActorParam->addBoundaryCondition(0));
	EXPECT_EQ(2u, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->addBoundaryCondition(10));
	EXPECT_EQ(3u, femActorParam->getBoundaryConditions().size());

	// Remove 1 by 1
	EXPECT_TRUE(femActorParam->removeBoundaryCondition(10));
	EXPECT_EQ(2u, femActorParam->getBoundaryConditions().size());
	EXPECT_FALSE(femActorParam->removeBoundaryCondition(5));
	EXPECT_EQ(2u, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->removeBoundaryCondition(0));
	EXPECT_EQ(1u, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->removeBoundaryCondition(1));
	EXPECT_EQ(0u, femActorParam->getBoundaryConditions().size());

	// Add all
	std::vector<unsigned int> data;
	data.push_back(0);
	data.push_back(1);
	data.push_back(2);
	data.push_back(3);
	data.push_back(2); // duplicate !
	EXPECT_EQ(4u, femActorParam->addBoundaryConditions(data));
	EXPECT_EQ(4u, femActorParam->getBoundaryConditions().size());

	// Clear all
	femActorParam->clearBoundaryConditions();
	EXPECT_EQ(0u, femActorParam->getBoundaryConditions().size());

	/// Boundary condition mass property
	femActorParam->setBoundaryConditionMass(0.0);
	EXPECT_NEAR(0.0, femActorParam->getBoundaryConditionMass(), epsilon);
	femActorParam->setBoundaryConditionMass(1.2);
	EXPECT_NEAR(1.2, femActorParam->getBoundaryConditionMass(), epsilon);
	femActorParam->setBoundaryConditionMass(0.0);
	EXPECT_NEAR(0.0, femActorParam->getBoundaryConditionMass(), epsilon);

	/// Boundary condition stiffness property
	femActorParam->setBoundaryConditionInverseMass(0.0);
	EXPECT_NEAR(0.0, femActorParam->getBoundaryConditionInverseMass(), epsilon);
	femActorParam->setBoundaryConditionInverseMass(1.2);
	EXPECT_NEAR(1.2, femActorParam->getBoundaryConditionInverseMass(), epsilon);
	femActorParam->setBoundaryConditionInverseMass(0.0);
	EXPECT_NEAR(0.0, femActorParam->getBoundaryConditionInverseMass(), epsilon);
}

}; // Physics

}; // SurgSim
