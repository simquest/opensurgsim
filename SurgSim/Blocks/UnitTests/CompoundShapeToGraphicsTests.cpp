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
#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Scene.h"

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