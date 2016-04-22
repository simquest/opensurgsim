// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/VisualizeConstraints.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Framework/FrameworkConvert.h"


#include <gtest/gtest.h>

namespace SurgSim
{

namespace Blocks
{

TEST(VisualizeConstraintsTest, Init)
{
	EXPECT_NO_THROW({ auto element = std::make_shared<VisualizeConstraints>(); });
	EXPECT_NO_THROW({ auto component = std::make_shared<VisualizeConstraintsBehavior>("Behavior"); });
}

TEST(VisualizeConstraintsTest, SingleSetter)
{
	auto visualizer = std::make_shared<VisualizeConstraintsBehavior>("Behavior");
	std::shared_ptr<Framework::Component> field =
		std::make_shared<Graphics::OsgVectorFieldRepresentation>("VectorField");
	EXPECT_NO_THROW(visualizer->setVectorField(Physics::CONSTRAINT_GROUP_TYPE_CONTACT, field));
	EXPECT_ANY_THROW(visualizer->setVectorField(Physics::CONSTRAINT_GROUP_TYPE_COUNT, field));
	EXPECT_ANY_THROW(visualizer->setVectorField(Physics::CONSTRAINT_GROUP_TYPE_CONTACT, nullptr));
}

TEST(VisualizeConstraintsTest, Serialization)
{
	auto behavior = std::make_shared<VisualizeConstraintsBehavior>("VisualizeConstraints");
	std::shared_ptr<Framework::Component> field =
		std::make_shared<Graphics::OsgVectorFieldRepresentation>("VectorField");

	VisualizeConstraintsBehavior::FieldsType fields;
	fields.push_back(std::make_pair(Physics::CONSTRAINT_GROUP_TYPE_CONTACT, field));
	EXPECT_NO_THROW(behavior->setValue("VectorFields", fields));

	EXPECT_EQ("SurgSim::Blocks::VisualizeConstraintsBehavior", behavior->getClassName());

	// Encode
	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior));
	EXPECT_TRUE(node.IsMap());

	// Decode
	std::shared_ptr<VisualizeConstraintsBehavior> newBehavior;
	EXPECT_NO_THROW(newBehavior = std::dynamic_pointer_cast<VisualizeConstraintsBehavior>(
									  node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_NE(nullptr, newBehavior);
	// Verify

	auto newFields = newBehavior->getValue<VisualizeConstraintsBehavior::FieldsType>("VectorFields");
	EXPECT_EQ(1u, newFields.size());
	EXPECT_EQ("VectorField", newFields[0].second->getName());
}



}
}