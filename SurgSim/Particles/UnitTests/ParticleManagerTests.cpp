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

#include "boost/thread/thread.hpp"

#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Particles/ParticleManager.h"
#include "SurgSim/Particles/UnitTests/MockObjects.h"


namespace SurgSim
{
namespace Particles
{

TEST(ParticleManagerTest, Constructor)
{
	std::shared_ptr<ParticleManager> particleManager;
	ASSERT_NO_THROW(particleManager = std::make_shared<ParticleManager>());
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_PARTICLES, particleManager->getType());
}

TEST(ParticleManagerTest, GetType)
{
	auto particleManager = std::make_shared<ParticleManager>();
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_PARTICLES, particleManager->getType());
}

TEST(ParticleManagerTest, AddComponents)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");
	auto behavior = std::make_shared<MockParticleBehavior>("ParticleBehavior");
	auto invalidComponent = std::make_shared<SurgSim::Input::InputComponent>("Invalid");

	auto emitter = std::make_shared<EmitterRepresentation>("Emitter");
	emitter->setTarget(particleSystem);
	emitter->setShape(std::make_shared<SurgSim::Math::SphereShape>(0.1));

	auto particleManager = std::make_shared<ParticleManager>();
	runtime->addManager(particleManager);

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Element");
	element->addComponent(emitter);
	element->addComponent(particleSystem);
	element->addComponent(behavior);
	element->addComponent(invalidComponent);
	runtime->getScene()->addSceneElement(element);

	EXPECT_TRUE(emitter->isInitialized());
	EXPECT_TRUE(particleSystem->isInitialized());
	EXPECT_TRUE(behavior->isInitialized());
	EXPECT_TRUE(invalidComponent->isInitialized());

	runtime->start(true);
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(10));

	EXPECT_TRUE(particleManager->isInitialized());
	EXPECT_TRUE(particleManager->isRunning());
	EXPECT_FALSE(particleManager->isIdle());
	EXPECT_TRUE(emitter->isAwake());
	EXPECT_TRUE(particleSystem->isAwake());
	EXPECT_TRUE(behavior->isAwake());
	EXPECT_FALSE(invalidComponent->isAwake());

	runtime->stop();
}

TEST(ParticleManagerTest, UpdateTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");
	auto behavior = std::make_shared<MockParticleBehavior>("ParticleBehavior");

	auto emitter = std::make_shared<MockEmitter>("Emitter");
	emitter->setTarget(particleSystem);
	emitter->setShape(std::make_shared<SurgSim::Math::SphereShape>(0.1));

	auto particleManager = std::make_shared<ParticleManager>();
	runtime->addManager(particleManager);

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Element");
	element->addComponent(emitter);
	element->addComponent(particleSystem);
	element->addComponent(behavior);
	runtime->getScene()->addSceneElement(element);

	emitter->setLocalActive(false);
	particleSystem->setLocalActive(false);
	behavior->setLocalActive(false);

	runtime->start(true);
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(10));

	EXPECT_EQ(0, emitter->updateCount);
	EXPECT_EQ(0, particleSystem->updateCount);
	EXPECT_EQ(0, behavior->updateCount);

	emitter->setLocalActive(true);
	particleSystem->setLocalActive(true);
	behavior->setLocalActive(true);

	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(10));

	EXPECT_LT(0, emitter->updateCount);
	EXPECT_LT(0, particleSystem->updateCount);
	EXPECT_LT(0, behavior->updateCount);

	runtime->stop();
}

}; // namespace Particles
}; // namespace SurgSim
