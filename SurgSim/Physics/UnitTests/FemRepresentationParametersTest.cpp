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

#include <SurgSim/Physics/FemRepresentationParameters.h>

namespace
{
	const double epsilon = 1e-10;
}

namespace SurgSim
{

namespace Physics
{

TEST(FemRepresentationParametersTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FemRepresentationParameters femRepresentationParam;});
}

TEST(FemRepresentationParametersTest, DefaultValueTest)
{
	// Create the base rigid representation state
	std::shared_ptr<FemRepresentationParameters> femRepresentationParam =
		std::make_shared<FemRepresentationParameters>();

	// Mass density [default = 0]
	EXPECT_NEAR(0.0, femRepresentationParam->getDensity(), epsilon);
	// Rayleigh damping mass parameter [default = 0]
	EXPECT_NEAR(0.0, femRepresentationParam->getRayleighDampingMass(), epsilon);
	// Rayleigh damping stiffness parameter [default = 0]
	EXPECT_NEAR(0.0, femRepresentationParam->getRayleighDampingStiffness(), epsilon);
	// Young modulus [default = 0]
	EXPECT_NEAR(0.0, femRepresentationParam->getYoungModulus(), epsilon);
	// Poisson ratio [default = 0]
	EXPECT_NEAR(0.0, femRepresentationParam->getPoissonRatio(), epsilon);
	// isValid [default = false]
	EXPECT_FALSE(femRepresentationParam->isValid());
}

TEST(FemRepresentationParametersTest, SetGetValidTest)
{
	// Create the base rigid representation state
	std::shared_ptr<FemRepresentationParameters> femRepresentationParam =
		std::make_shared<FemRepresentationParameters>();

	// Set proper density, young modulus and poisson ratio to test validy flag
	femRepresentationParam->setDensity(12.52);
	EXPECT_NEAR(12.52, femRepresentationParam->getDensity(), epsilon);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setYoungModulus(10e7);
	EXPECT_NEAR(10e7, femRepresentationParam->getYoungModulus(), epsilon);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(0.4);
	EXPECT_NEAR(0.4, femRepresentationParam->getPoissonRatio(), epsilon);
	EXPECT_TRUE(femRepresentationParam->isValid());

	// Test Rayleigh damping mass
	femRepresentationParam->setRayleighDampingMass(13.21);
	EXPECT_NEAR(13.21, femRepresentationParam->getRayleighDampingMass(), epsilon);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setRayleighDampingMass(-13.21);
	EXPECT_NEAR(-13.21, femRepresentationParam->getRayleighDampingMass(), epsilon);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setRayleighDampingMass(0.0);
	EXPECT_NEAR(0.0, femRepresentationParam->getRayleighDampingMass(), epsilon);
	EXPECT_TRUE(femRepresentationParam->isValid());

	// Test Rayleigh damping stiffness
	femRepresentationParam->setRayleighDampingStiffness(3.87);
	EXPECT_NEAR(3.87, femRepresentationParam->getRayleighDampingStiffness(), epsilon);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setRayleighDampingStiffness(-3.87);
	EXPECT_NEAR(-3.87, femRepresentationParam->getRayleighDampingStiffness(), epsilon);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setRayleighDampingStiffness(0.0);
	EXPECT_NEAR(0.0, femRepresentationParam->getRayleighDampingStiffness(), epsilon);
	EXPECT_TRUE(femRepresentationParam->isValid());

	// Test validity flag some more
	femRepresentationParam->setDensity(0.0);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setDensity(1.0);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setYoungModulus(0.0);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setYoungModulus(1.0);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(-2.0);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(-1.0);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(-0.99);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(0.0);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(0.499);
	EXPECT_TRUE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(0.5);
	EXPECT_FALSE(femRepresentationParam->isValid());
	femRepresentationParam->setPoissonRatio(1.0);
	EXPECT_FALSE(femRepresentationParam->isValid());
}


TEST(FemRepresentationParametersTest, BoundaryConditionsTest)
{
	// Create the base rigid representation state
	std::shared_ptr<FemRepresentationParameters> femRepresentationParam =
		std::make_shared<FemRepresentationParameters>();

	// Add 1 by 1
	EXPECT_EQ(0u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_TRUE(femRepresentationParam->addBoundaryCondition(0));
	EXPECT_EQ(1u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_TRUE(femRepresentationParam->addBoundaryCondition(1));
	EXPECT_EQ(2u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_FALSE(femRepresentationParam->addBoundaryCondition(0));
	EXPECT_EQ(2u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_TRUE(femRepresentationParam->addBoundaryCondition(10));
	EXPECT_EQ(3u, femRepresentationParam->getBoundaryConditions().size());

	// Remove 1 by 1
	EXPECT_TRUE(femRepresentationParam->removeBoundaryCondition(10));
	EXPECT_EQ(2u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_FALSE(femRepresentationParam->removeBoundaryCondition(5));
	EXPECT_EQ(2u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_TRUE(femRepresentationParam->removeBoundaryCondition(0));
	EXPECT_EQ(1u, femRepresentationParam->getBoundaryConditions().size());
	EXPECT_TRUE(femRepresentationParam->removeBoundaryCondition(1));
	EXPECT_EQ(0u, femRepresentationParam->getBoundaryConditions().size());

	// Add all
	std::vector<unsigned int> data;
	data.push_back(0);
	data.push_back(1);
	data.push_back(2);
	data.push_back(3);
	data.push_back(2); // duplicate !
	EXPECT_EQ(4u, femRepresentationParam->addBoundaryConditions(data));
	EXPECT_EQ(4u, femRepresentationParam->getBoundaryConditions().size());

	// Clear all
	femRepresentationParam->clearBoundaryConditions();
	EXPECT_EQ(0u, femRepresentationParam->getBoundaryConditions().size());

	/// Boundary condition mass property
	femRepresentationParam->setBoundaryConditionMass(0.0);
	EXPECT_NEAR(0.0, femRepresentationParam->getBoundaryConditionMass(), epsilon);
	femRepresentationParam->setBoundaryConditionMass(1.2);
	EXPECT_NEAR(1.2, femRepresentationParam->getBoundaryConditionMass(), epsilon);
	femRepresentationParam->setBoundaryConditionMass(0.0);
	EXPECT_NEAR(0.0, femRepresentationParam->getBoundaryConditionMass(), epsilon);

	/// Boundary condition stiffness property
	femRepresentationParam->setBoundaryConditionInverseMass(0.0);
	EXPECT_NEAR(0.0, femRepresentationParam->getBoundaryConditionInverseMass(), epsilon);
	femRepresentationParam->setBoundaryConditionInverseMass(1.2);
	EXPECT_NEAR(1.2, femRepresentationParam->getBoundaryConditionInverseMass(), epsilon);
	femRepresentationParam->setBoundaryConditionInverseMass(0.0);
	EXPECT_NEAR(0.0, femRepresentationParam->getBoundaryConditionInverseMass(), epsilon);
}

}; // Physics

}; // SurgSim
