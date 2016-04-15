// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/CompoundShapeToGraphics.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Physics/FixedRepresentation.h"

#include <gtest/gtest.h>

namespace SurgSim
{

namespace Blocks
{

TEST(CompoundShapeToGraphicsTests, Init)
{
	EXPECT_NO_THROW(std::make_shared<CompoundShapeToGraphics>("Copier"));
}

TEST(CompoundShapeToGraphicsTests, SetShape)
{
	auto shape = std::make_shared<Math::CompoundShape>();
	auto behavior = std::make_shared<CompoundShapeToGraphics>("Copier");

	EXPECT_EQ(nullptr, behavior->getShape());
	EXPECT_NO_THROW(behavior->setShape(shape));
	EXPECT_EQ(shape, behavior->getShape());

	// With shape set, setting Source will fail.
	auto physics = std::make_shared<Physics::FixedRepresentation>("Physics");
	EXPECT_ANY_THROW(behavior->setSource(physics));
}

TEST(CompoundShapeToGraphicsTests, SetSource)
{
	auto behavior = std::make_shared<CompoundShapeToGraphics>("Copier");
	auto physics = std::make_shared<Physics::FixedRepresentation>("Physics");
	auto collisions = std::make_shared<Collision::ShapeCollisionRepresentation>("Physics");
	auto graphics = std::make_shared<Graphics::OsgSceneryRepresentation>("Graphics");

	EXPECT_NO_THROW(behavior->setSource(physics));
	EXPECT_EQ(physics, behavior->getSource());
	EXPECT_NO_THROW(behavior->setSource(collisions));
	EXPECT_EQ(collisions, behavior->getSource());
	EXPECT_ANY_THROW(behavior->setSource(graphics));

	// With Source being set, setting Shape will fail.
	auto shape = std::make_shared<Math::CompoundShape>();
	EXPECT_ANY_THROW(behavior->setShape(shape));
}

TEST(CompoundShapeToGraphicsTests, Serialization)
{
	std::shared_ptr<SurgSim::Framework::Component> behavior;
	EXPECT_NO_THROW(
		behavior =
		SurgSim::Framework::Component::getFactory().create("SurgSim::Blocks::CompoundShapeToGraphics", "newCopier"));
	auto behavior2 = std::make_shared<CompoundShapeToGraphics>("Copier");
	auto graphics = std::make_shared<Graphics::OsgSceneryRepresentation>("Graphics");
	std::shared_ptr<Math::Shape> compoundShape = std::make_shared<Math::CompoundShape>();
	std::shared_ptr<Framework::Component> physics = std::make_shared<Physics::FixedRepresentation>("Physics");

	std::vector<std::shared_ptr<Framework::Component>> targets;
	targets.push_back(graphics);
	targets.push_back(graphics);
	targets.push_back(graphics);

	EXPECT_NO_THROW(behavior->setValue("Targets", targets));
	EXPECT_NO_THROW(behavior->setValue("Source", physics));
	EXPECT_NO_THROW(behavior2->setValue("Targets", targets));
	EXPECT_NO_THROW(behavior2->setValue("Shape", compoundShape));

	EXPECT_EQ(3u, behavior->getValue<std::vector<std::shared_ptr<Framework::Component>>>("Targets").size());
	EXPECT_EQ(physics, behavior->getValue<std::shared_ptr<Framework::Component>>("Source"));
	EXPECT_EQ(compoundShape, behavior2->getValue<std::shared_ptr<Math::CompoundShape>>("Shape"));

	YAML::Node node;
	// Shape is not set on 'behavior', serialization should fail.
	EXPECT_ANY_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior));
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior2));
	EXPECT_EQ(6u, node[behavior2->getClassName()].size());

	std::shared_ptr<SurgSim::Blocks::CompoundShapeToGraphics> newBehavior;
	EXPECT_NO_THROW(newBehavior = std::dynamic_pointer_cast<SurgSim::Blocks::CompoundShapeToGraphics>(
															node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	EXPECT_EQ(3u, newBehavior->getValue<std::vector<std::shared_ptr<Framework::Component>>>("Targets").size());
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<Math::CompoundShape>>("Shape"));
}

TEST(CompoundShapeToGraphicsTests, SetGraphics)
{
	auto behavior = std::make_shared<CompoundShapeToGraphics>("Copier");

	EXPECT_EQ(0u, behavior->getTargets().size());

	auto graphics = std::make_shared<Graphics::OsgSceneryRepresentation>("One");
	auto physics = std::make_shared<Physics::FixedRepresentation>("Invalid");

	EXPECT_NO_THROW(behavior->addTarget(graphics));
	EXPECT_EQ(1u, behavior->getTargets().size());

	EXPECT_ANY_THROW(behavior->addTarget(physics));
	EXPECT_EQ(1u, behavior->getTargets().size());

	std::vector<std::shared_ptr<Framework::Component>> targets;
	targets.push_back(graphics);
	targets.push_back(graphics);
	targets.push_back(graphics);

	EXPECT_NO_THROW(behavior->setTargets(targets));
	EXPECT_EQ(3u, behavior->getTargets().size());
}

TEST(CompoundShapeToGraphicsTests, SetSourceWithValidShape)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto element = std::make_shared<Framework::BasicSceneElement>("Element");

	auto behavior = std::make_shared<CompoundShapeToGraphics>("Copier");

	auto physics = std::make_shared<Physics::FixedRepresentation>("Physics");

	auto shape = std::make_shared<Math::CompoundShape>();

	physics->setShape(shape);
	behavior->setSource(physics);

	element->addComponent(physics);
	element->addComponent(behavior);
	runtime->getScene()->addSceneElement(element);

	EXPECT_NO_THROW(behavior->wakeUp());
	EXPECT_EQ(shape, behavior->getShape());
}

TEST(CompoundShapeToGraphicsTests, SetSourceWithInvalidShape)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto element = std::make_shared<Framework::BasicSceneElement>("Element");

	auto behavior = std::make_shared<CompoundShapeToGraphics>("Copier");

	auto collision = std::make_shared<Collision::ShapeCollisionRepresentation>("Collisions");

	auto shape = std::make_shared<Math::BoxShape>();
	EXPECT_ANY_THROW(behavior->setShape(shape));

	collision->setShape(shape);
	behavior->setSource(collision);

	element->addComponent(collision);
	element->addComponent(behavior);
	runtime->getScene()->addSceneElement(element);

	EXPECT_ANY_THROW(behavior->wakeUp());
}

TEST(CompoundShapeToGraphicsTests, SetSourceWithoutShape)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto element = std::make_shared<Framework::BasicSceneElement>("Element");

	auto behavior = std::make_shared<CompoundShapeToGraphics>("Copier");

	auto collision = std::make_shared<Collision::ShapeCollisionRepresentation>("Collisions");

	behavior->setSource(collision);

	element->addComponent(collision);
	element->addComponent(behavior);
	runtime->getScene()->addSceneElement(element);

	EXPECT_ANY_THROW(behavior->wakeUp());
}

}
}