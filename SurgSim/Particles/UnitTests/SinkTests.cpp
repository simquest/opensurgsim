// This file is a part of the OpenSurgSim project.
// Copyright 2015-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"
#include "SurgSim/Particles/Sink.h"
#include "SurgSim/Particles/UnitTests/MockObjects.h"
#include "SurgSim/Physics/PhysicsManager.h"


namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, MockParticleSystem, MockParticleSystem);
}

namespace SurgSim
{
namespace Particles
{

TEST(SinkTest, Constructor)
{
	std::shared_ptr<Sink> sink;
	ASSERT_NO_THROW(sink = std::make_shared<Sink>("Sink"));

	EXPECT_EQ(Framework::MANAGER_TYPE_PHYSICS, sink->getTargetManagerType());
	EXPECT_EQ(nullptr, sink->getTarget());
	EXPECT_EQ(nullptr, sink->getCollisionRepresentation());
}

TEST(SinkTest, SetGetCollisionRepresentation)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto collision = std::make_shared<Collision::ShapeCollisionRepresentation>("Collision");
	auto notCollision = std::make_shared<Sink>("Not a CollisionRepresentation");
	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");

	{
		auto sink = std::make_shared<Sink>("Sink");
		sink->setTarget(particleSystem);
		EXPECT_THROW(sink->setCollisionRepresentation(notCollision), SurgSim::Framework::AssertionFailure);
		EXPECT_TRUE(sink->initialize(runtime));
		EXPECT_FALSE(sink->wakeUp()) << "Without a CollisionRepresentation, the sink should not wakup";
	}
	{
		auto sink = std::make_shared<Sink>("Sink");
		sink->setTarget(particleSystem);
		EXPECT_NO_THROW(sink->setCollisionRepresentation(collision));
		EXPECT_EQ(collision, sink->getCollisionRepresentation());
		EXPECT_TRUE(sink->initialize(runtime));
		EXPECT_TRUE(sink->wakeUp()) << "With a CollisionRepresentation, the sink should wakeup";
	}
}

TEST(SinkTest, SetGetTarget)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto collision = std::make_shared<Collision::ShapeCollisionRepresentation>("Collision");
	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");
	auto notParticleSystem = std::make_shared<Sink>("Not a ParticleSystem");

	{
		auto sink = std::make_shared<Sink>("Sink");
		sink->setCollisionRepresentation(collision);
		EXPECT_THROW(sink->setTarget(notParticleSystem), SurgSim::Framework::AssertionFailure);
		EXPECT_TRUE(sink->initialize(runtime));
		EXPECT_FALSE(sink->wakeUp()) << "Without a target, the sink should not wakup";
	}
	{
		auto sink = std::make_shared<Sink>("Sink");
		sink->setCollisionRepresentation(collision);
		EXPECT_NO_THROW(sink->setTarget(particleSystem));
		EXPECT_EQ(particleSystem, sink->getTarget());
		EXPECT_TRUE(sink->initialize(runtime));
		EXPECT_TRUE(sink->wakeUp()) << "With a target, the sink should wakeup";
	}
}

TEST(SinkTest, Update)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	runtime->addManager(std::make_shared<Physics::PhysicsManager>());

	auto element = std::make_shared<Framework::BasicSceneElement>("Element");

	auto particleCollision = std::make_shared<ParticlesCollisionRepresentation>("Particle Collision");
	particleCollision->setParticleRadius(0.01);
	element->addComponent(particleCollision);

	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");
	particleSystem->setMaxParticles(10);
	particleSystem->setCollisionRepresentation(particleCollision);
	element->addComponent(particleSystem);

	auto mesh = std::make_shared<Math::MeshShape>();
	mesh->load("Geometry/Cube.ply");

	auto sinkCollision = std::make_shared<Collision::ShapeCollisionRepresentation>("Sink Collision");
	sinkCollision->setShape(mesh);
	element->addComponent(sinkCollision);

	auto sink = std::make_shared<Sink>("Sink");
	sink->setTarget(particleSystem);
	sink->setCollisionRepresentation(sinkCollision);
	element->addComponent(sink);

	particleSystem->addParticle(Math::Vector3d(0.0, 1.009, 0.0), Math::Vector3d::Zero(), 10);
	runtime->getScene()->addSceneElement(element);
	EXPECT_EQ(1, particleSystem->getParticles().safeGet()->getNumVertices());

	runtime->start(true);
	runtime->step();
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	runtime->stop();

	EXPECT_EQ(0, particleSystem->getParticles().safeGet()->getNumVertices());
}

TEST(SinkTest, Serialization)
{
	auto sink = std::make_shared<Sink>("Sink");
	EXPECT_EQ("SurgSim::Particles::Sink", sink->getClassName());

	std::shared_ptr<Framework::Component> expectedTarget = std::make_shared<MockParticleSystem>("ParticleSystem");
	std::shared_ptr<Framework::Component> expectedCollision =
		std::make_shared<Collision::ShapeCollisionRepresentation>("Collision");

	EXPECT_NO_THROW(sink->setValue("CollisionRepresentation", expectedCollision));
	EXPECT_NO_THROW(sink->setValue("Target", expectedTarget));

	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<Framework::Component>::encode(*sink));
	EXPECT_TRUE(node.IsMap());

	std::shared_ptr<Sink> decodedSink;
	ASSERT_NO_THROW(decodedSink = std::dynamic_pointer_cast<Sink>(node.as<std::shared_ptr<Framework::Component>>()));

	EXPECT_EQ("ParticleSystem", decodedSink->getValue<std::shared_ptr<Framework::Component>>("Target")->getName());
	EXPECT_EQ("Collision",
			decodedSink->getValue<std::shared_ptr<Framework::Component>>("CollisionRepresentation")->getName());
}

}; // namespace Particles
}; // namespace SurgSim
