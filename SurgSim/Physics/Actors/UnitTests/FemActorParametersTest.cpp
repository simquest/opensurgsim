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

#include <SurgSim/Physics/Actors/FemActorParameters.h>

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
	EXPECT_EQ(0.0, femActorParam->getDensity());
	// Rayleigh damping mass parameter [default = 0]
	EXPECT_EQ(0.0, femActorParam->getRayleighDampingMass());
	// Rayleigh damping stiffness parameter [default = 0]
	EXPECT_EQ(0.0, femActorParam->getRayleighDampingStiffness());
	// Young modulus [default = 0]
	EXPECT_EQ(0.0, femActorParam->getYoungModulus());
	// Poisson ratio [default = 0]
	EXPECT_EQ(0.0, femActorParam->getPoissonRatio());
	// isValid [default = false]
	EXPECT_FALSE(femActorParam->isValid());
}

TEST(FemActorParametersTest, SetGetValidTest)
{
	// Create the base rigid actor state
	std::shared_ptr<FemActorParameters> femActorParam = std::make_shared<FemActorParameters>();

	// Set proper density, young modulus and poisson ratio to test validy flag
	femActorParam->setDensity(12.52);
	EXPECT_EQ(12.52, femActorParam->getDensity());
	EXPECT_FALSE(femActorParam->isValid());
	femActorParam->setYoungModulus(10e7);
	EXPECT_EQ(10e7, femActorParam->getYoungModulus());
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(0.4);
	EXPECT_EQ(0.4, femActorParam->getPoissonRatio());
	EXPECT_TRUE(femActorParam->isValid());

	// Test Rayleigh damping mass
	femActorParam->setRayleighDampingMass(13.21);
	EXPECT_EQ(13.21, femActorParam->getRayleighDampingMass());
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setRayleighDampingMass(0.0);
	EXPECT_EQ(0.0, femActorParam->getRayleighDampingMass());
	EXPECT_TRUE(femActorParam->isValid());

	// Test Rayleigh damping stiffness
	femActorParam->setRayleighDampingStiffness(3.87);
	EXPECT_EQ(3.87, femActorParam->getRayleighDampingStiffness());
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setRayleighDampingStiffness(0.0);
	EXPECT_EQ(0.0, femActorParam->getRayleighDampingStiffness());
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
	femActorParam->setPoissonRatio(0.0);
	EXPECT_TRUE(femActorParam->isValid());
	femActorParam->setPoissonRatio(0.4);
	EXPECT_TRUE(femActorParam->isValid());

	// Test ASSERT
	EXPECT_THROW(femActorParam->setPoissonRatio(-1.5), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(femActorParam->setPoissonRatio(-1.0), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(femActorParam->setPoissonRatio(-0.99999));
	EXPECT_NO_THROW(femActorParam->setPoissonRatio(-0.5));
	EXPECT_NO_THROW(femActorParam->setPoissonRatio(0.0));
	EXPECT_NO_THROW(femActorParam->setPoissonRatio(0.3));
	EXPECT_NO_THROW(femActorParam->setPoissonRatio(0.49999));
	EXPECT_THROW(femActorParam->setPoissonRatio(0.5), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(femActorParam->setPoissonRatio(1.0), SurgSim::Framework::AssertionFailure);
}


TEST(FemActorParametersTest, BoundaryConditionsTest)
{
	// Create the base rigid actor state
	std::shared_ptr<FemActorParameters> femActorParam = std::make_shared<FemActorParameters>();

	// Add 1 by 1
	EXPECT_EQ(0, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->addBoundaryCondition(0));
	EXPECT_EQ(1, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->addBoundaryCondition(1));
	EXPECT_EQ(2, femActorParam->getBoundaryConditions().size());
	EXPECT_FALSE(femActorParam->addBoundaryCondition(0));
	EXPECT_EQ(2, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->addBoundaryCondition(10));
	EXPECT_EQ(3, femActorParam->getBoundaryConditions().size());

	// Remove 1 by 1
	EXPECT_TRUE(femActorParam->removeBoundaryCondition(10));
	EXPECT_EQ(2, femActorParam->getBoundaryConditions().size());
	EXPECT_FALSE(femActorParam->removeBoundaryCondition(5));
	EXPECT_EQ(2, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->removeBoundaryCondition(0));
	EXPECT_EQ(1, femActorParam->getBoundaryConditions().size());
	EXPECT_TRUE(femActorParam->removeBoundaryCondition(1));
	EXPECT_EQ(0, femActorParam->getBoundaryConditions().size());

	// Add all
	std::vector<unsigned int> data;
	data.push_back(0);
	data.push_back(1);
	data.push_back(2);
	data.push_back(3);
	data.push_back(2); // duplicate !
	EXPECT_EQ(4u, femActorParam->addBoundaryConditions(data));
	EXPECT_EQ(4, femActorParam->getBoundaryConditions().size());

	// Clear all
	femActorParam->clearBoundaryConditions();
	EXPECT_EQ(0, femActorParam->getBoundaryConditions().size());

	/// Boundary condition mass property
	femActorParam->setBoundaryConditionMass(0.0);
	EXPECT_EQ(0.0, femActorParam->getBoundaryConditionMass());
	femActorParam->setBoundaryConditionMass(1.2);
	EXPECT_EQ(1.2, femActorParam->getBoundaryConditionMass());
	femActorParam->setBoundaryConditionMass(0.0);
	EXPECT_EQ(0.0, femActorParam->getBoundaryConditionMass());

	/// Boundary condition stiffness property
	femActorParam->setBoundaryConditionInverseMass(0.0);
	EXPECT_EQ(0.0, femActorParam->getBoundaryConditionInverseMass());
	femActorParam->setBoundaryConditionInverseMass(1.2);
	EXPECT_EQ(1.2, femActorParam->getBoundaryConditionInverseMass());
	femActorParam->setBoundaryConditionInverseMass(0.0);
	EXPECT_EQ(0.0, femActorParam->getBoundaryConditionInverseMass());
}

}; // Physics

}; // SurgSim
