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
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/SphRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Blocks::TransferParticlesToPointCloudBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Particles::Particle;
using SurgSim::Particles::ParticleReference;
using SurgSim::Particles::ParticleSystemRepresentation;
using SurgSim::Particles::SphRepresentation;
using SurgSim::Physics::RigidRepresentation;

TEST(TransferParticlesToPointCloudBehaviorTests, ConstructorTest)
{
	ASSERT_NO_THROW(TransferParticlesToPointCloudBehavior("TestBehavior"));
}

TEST(TransferParticlesToPointCloudBehaviorTests, SetGetSourceTest)
{
	auto particles  = std::make_shared<SphRepresentation>("Particles");
	auto rigid = std::make_shared<RigidRepresentation>("Rigid");
	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");

	EXPECT_THROW(behavior->setSource(nullptr), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(behavior->setSource(rigid), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setSource(particles));
	EXPECT_EQ(particles, behavior->getSource());
}

TEST(TransferParticlesToPointCloudBehaviorTests, SetGetTargetTest)
{
	auto pointCloud = std::make_shared<OsgPointCloudRepresentation>("OsgMesh");
	auto graphicsBox = std::make_shared<OsgBoxRepresentation>("OsgBox");
	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");

	EXPECT_THROW(behavior->setTarget(nullptr), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(behavior->setTarget(graphicsBox), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(behavior->setTarget(pointCloud));
	EXPECT_EQ(pointCloud, behavior->getTarget());
}

TEST(TransferParticlesToPointCloudBehaviorTests, UpdateTest)
{
	auto runtime = std::make_shared<Runtime>("config.txt");
	auto behaviorManager = std::make_shared<BehaviorManager>();
	runtime->addManager(behaviorManager);

	auto scene = runtime->getScene();
	auto sceneElement = std::make_shared<BasicSceneElement>("scene element");
	auto particles = std::make_shared<SphRepresentation>("Particles");
	auto pointCloud = std::make_shared<OsgPointCloudRepresentation>("GraphicsMesh");
	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");
	particles->setMaxParticles(10);
	particles->setMassPerParticle(1.0);
	particles->setDensityReference(1.0);
	particles->setGasStiffness(1.0);
	particles->setKernelSupport(1.0);
	behavior->setSource(particles);
	behavior->setTarget(pointCloud);
	sceneElement->addComponent(behavior);
	sceneElement->addComponent(particles);
	sceneElement->addComponent(pointCloud);
	scene->addSceneElement(sceneElement);

	for (size_t particleId = 0; particleId < 10; particleId++)
	{
		Particle p;
		p.setLifetime(100000);
		p.setPosition(Vector3d(static_cast<double>(particleId), 0.0, 0.0));
		p.setVelocity(Vector3d::Zero());
		particles->addParticle(p);
	}

	// Test doInitialize(), doWakeUP()
	EXPECT_NO_THROW(runtime->start());
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));

	auto& allParticles = particles->getParticleReferences();
	auto target = pointCloud->getVertices();
	ASSERT_NE(0, target->getNumVertices());
	ASSERT_NE(0, allParticles.size());
	ASSERT_EQ(particles->getMaxParticles(), target->getNumVertices());

	size_t nodeId = 0;
	for (std::list<ParticleReference>::iterator particle = allParticles.begin();
		particle != allParticles.end();
		particle++)
	{
		EXPECT_TRUE(particle->getPosition().isApprox(target->getVertex(nodeId).position));
		nodeId++;
	}
	for (; nodeId < particles->getMaxParticles(); nodeId++)
	{
		EXPECT_TRUE(target->getVertex(nodeId).position.isZero());
		nodeId++;
	}

	// Test TransferParticlesToGraphicsBehavior::update()
	particles->removeParticle(allParticles.front());
	particles->removeParticle(allParticles.back());
	behavior->update(1.0);

	nodeId = 0;
	for (std::list<ParticleReference>::const_iterator particle = allParticles.cbegin();
		particle != allParticles.cend();
		particle++)
	{
		EXPECT_TRUE(particle->getPosition().isApprox(target->getVertex(nodeId).position));
		nodeId++;
	}
	for (; nodeId < particles->getMaxParticles(); nodeId++)
	{
		EXPECT_TRUE(target->getVertex(nodeId).position.isZero());
		nodeId++;
	}

	runtime->stop();
}

TEST(TransferParticlesToPointCloudBehaviorTests, SerializationTest)
{
	std::shared_ptr<SurgSim::Framework::Component> particles = std::make_shared<SphRepresentation>("Particles");
	std::shared_ptr<SurgSim::Framework::Component> pointCloud =
		std::make_shared<OsgPointCloudRepresentation>("GraphicsMesh");

	auto behavior = std::make_shared<TransferParticlesToPointCloudBehavior>("Behavior");

	EXPECT_NO_THROW(behavior->setValue("Source", particles));
	EXPECT_NO_THROW(behavior->setValue("Target", pointCloud));

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*behavior));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Blocks::TransferParticlesToPointCloudBehavior"];
	EXPECT_EQ(5u, data.size());

	std::shared_ptr<TransferParticlesToPointCloudBehavior> newBehavior;
	std::shared_ptr<SurgSim::Framework::Component> nodeAsComponent =
		node.as<std::shared_ptr<SurgSim::Framework::Component>>();
	ASSERT_NO_THROW(newBehavior = std::dynamic_pointer_cast<TransferParticlesToPointCloudBehavior>(nodeAsComponent));

	EXPECT_EQ("SurgSim::Blocks::TransferParticlesToPointCloudBehavior", newBehavior->getClassName());
	EXPECT_NE(nullptr,
		newBehavior->getValue<std::shared_ptr<SurgSim::Particles::ParticleSystemRepresentation>>("Source"));
	EXPECT_NE(nullptr,
		newBehavior->getValue<std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation>>("Target"));
}
