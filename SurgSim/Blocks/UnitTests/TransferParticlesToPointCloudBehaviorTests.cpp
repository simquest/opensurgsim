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

/// \file
/// Tests for the TransferParticlesToGraphicsBehavior class.

#include <gtest/gtest.h>

#include "SurgSim/Blocks/TransferParticlesToPointCloudBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Particles/SphRepresentation.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Blocks
{

TEST(TransferParticlesToPointCloudBehaviorTests, ConstructorTest)
{
	ASSERT_NO_THROW(TransferParticlesToPointCloudBehavior("TestBehavior"));
}

TEST(TransferParticlesToPointCloudBehaviorTests, SetGetSourceTest)
{
	auto particles  = std::make_shared<Particles::SphRepresentation>("Particles");
	auto rigid = std::make_shared<Physics::RigidRepresentation>("Rigid");
	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");

	EXPECT_THROW(behavior->setSource(nullptr), Framework::AssertionFailure);
	EXPECT_THROW(behavior->setSource(rigid), Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setSource(particles));
	EXPECT_EQ(particles, behavior->getSource());
}

TEST(TransferParticlesToPointCloudBehaviorTests, SetGetTargetTest)
{
	auto pointCloud = std::make_shared<Graphics::OsgPointCloudRepresentation>("OsgMesh");
	auto graphicsBox = std::make_shared<Graphics::OsgBoxRepresentation>("OsgBox");
	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");

	EXPECT_THROW(behavior->setTarget(nullptr), Framework::AssertionFailure);
	EXPECT_THROW(behavior->setTarget(graphicsBox), Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setTarget(pointCloud));
	EXPECT_EQ(pointCloud, behavior->getTarget());
}

TEST(TransferParticlesToPointCloudBehaviorTests, WakeUpTest)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");

	auto particles = std::make_shared<Particles::SphRepresentation>("Particles");
	particles->setMassPerParticle(1.0);
	particles->setDensity(1.0);
	particles->setGasStiffness(1.0);
	particles->setKernelSupport(1.0);

	auto pointCloud = std::make_shared<Graphics::OsgPointCloudRepresentation>("Graphics");

	{
		auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");
		EXPECT_TRUE(behavior->initialize(runtime));
		EXPECT_FALSE(behavior->wakeUp());
	}

	{
		auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");
		behavior->setTarget(pointCloud);
		EXPECT_TRUE(behavior->initialize(runtime));
		EXPECT_FALSE(behavior->wakeUp());
	}

	{
		auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");
		behavior->setSource(particles);
		EXPECT_TRUE(behavior->initialize(runtime));
		EXPECT_FALSE(behavior->wakeUp());
	}

	{
		auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");
		behavior->setSource(particles);
		behavior->setTarget(pointCloud);
		EXPECT_TRUE(behavior->initialize(runtime));
		EXPECT_TRUE(behavior->wakeUp());
	}
}

TEST(TransferParticlesToPointCloudBehaviorTests, UpdateTest)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	runtime->addManager(std::make_shared<Framework::BehaviorManager>());
	runtime->addManager(std::make_shared<Physics::PhysicsManager>());

	auto sceneElement = std::make_shared<Framework::BasicSceneElement>("Element");

	auto particles = std::make_shared<Particles::SphRepresentation>("Particles");
	particles->setMaxParticles(10);
	particles->setMassPerParticle(1.0);
	particles->setDensity(1.0);
	particles->setGasStiffness(1.0);
	particles->setKernelSupport(1.0);
	for (size_t particleId = 0; particleId < 10; particleId++)
	{
		particles->addParticle(Vector3d(static_cast<double>(particleId), 0.0, 0.0), Vector3d::Zero(), 100000);
	}
	sceneElement->addComponent(particles);

	auto pointCloud = std::make_shared<Graphics::OsgPointCloudRepresentation>("Graphics");
	sceneElement->addComponent(pointCloud);

	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");
	behavior->setSource(particles);
	behavior->setTarget(pointCloud);
	sceneElement->addComponent(behavior);

	auto scene = runtime->getScene();
	scene->addSceneElement(sceneElement);

	particles->update(0.1);
	behavior->update(0.1);
	auto sourceVertices = particles->getParticles().safeGet()->getVertices();
	auto targetVertices = pointCloud->getVertices()->getVertices();
	ASSERT_EQ(sourceVertices.size(), targetVertices.size());

	auto sourceVertex = sourceVertices.begin();
	auto targetVertex = targetVertices.begin();
	for (; sourceVertex != sourceVertices.end(); ++sourceVertex, ++targetVertex)
	{
		EXPECT_TRUE(sourceVertex->position.isApprox(targetVertex->position));
	}

	particles->removeParticle(0);
	particles->removeParticle(1);
	particles->update(0.1);
	behavior->update(0.1);

	sourceVertices = particles->getParticles().safeGet()->getVertices();
	targetVertices = pointCloud->getVertices()->getVertices();
	ASSERT_EQ(sourceVertices.size(), targetVertices.size());

	sourceVertex = sourceVertices.begin();
	targetVertex = targetVertices.begin();
	for (; sourceVertex != sourceVertices.end(); ++sourceVertex, ++targetVertex)
	{
		EXPECT_TRUE(sourceVertex->position.isApprox(targetVertex->position));
	}
}

TEST(TransferParticlesToPointCloudBehaviorTests, SerializationTest)
{
	std::shared_ptr<Framework::Component> particles = std::make_shared<Particles::SphRepresentation>("Particles");
	std::shared_ptr<Framework::Component> pointCloud =
		std::make_shared<Graphics::OsgPointCloudRepresentation>("Graphics");

	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");

	EXPECT_NO_THROW(behavior->setValue("Source", particles));
	EXPECT_NO_THROW(behavior->setValue("Target", pointCloud));

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<Framework::Component>::encode(*behavior));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Blocks::TransferParticlesToPointCloudBehavior"];
	EXPECT_EQ(5u, data.size());

	std::shared_ptr<TransferParticlesToPointCloudBehavior> newBehavior;
	std::shared_ptr<Framework::Component> nodeAsComponent = node.as<std::shared_ptr<Framework::Component>>();
	ASSERT_NO_THROW(newBehavior = std::dynamic_pointer_cast<TransferParticlesToPointCloudBehavior>(nodeAsComponent));

	EXPECT_EQ("SurgSim::Blocks::TransferParticlesToPointCloudBehavior", newBehavior->getClassName());
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<Particles::Representation>>("Source"));
	EXPECT_NE(nullptr, newBehavior->getValue<std::shared_ptr<Graphics::PointCloudRepresentation>>("Target"));
}

}; // namespace Blocks
}; // namespace SurgSim
