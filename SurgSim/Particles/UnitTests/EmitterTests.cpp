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

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Emitter.h"
#include "SurgSim/Particles/UnitTests/MockObjects.h"

using SurgSim::Framework::Component;
using SurgSim::Math::Shape;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, MockParticleSystem, MockParticleSystem);
}

namespace SurgSim
{
namespace Particles
{

TEST(EmitterTest, Constructor)
{
	std::shared_ptr<Emitter> emitter;
	ASSERT_NO_THROW(emitter = std::make_shared<Emitter>("Emitter"));

	EXPECT_EQ(EMIT_MODE_VOLUME, emitter->getMode());
	EXPECT_EQ(0.0, emitter->getRate());
	EXPECT_EQ(0.0, emitter->getLifetimeRange().first);
	EXPECT_EQ(0.0, emitter->getLifetimeRange().second);
	EXPECT_TRUE(emitter->getVelocityRange().first.isZero());
	EXPECT_TRUE(emitter->getVelocityRange().second.isZero());
	EXPECT_EQ(nullptr, emitter->getShape());
}

TEST(EmitterTest, SetGetShape)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto sphere = std::make_shared<SurgSim::Math::SphereShape>(0.1);
	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");

	{
		auto emitter = std::make_shared<Emitter>("Emitter");
		emitter->setTarget(particleSystem);
		EXPECT_TRUE(emitter->initialize(runtime));
		EXPECT_FALSE(emitter->wakeUp()) << "Without a shape, the emitter should not wakup";
	}
	{
		auto emitter = std::make_shared<Emitter>("Emitter");
		emitter->setTarget(particleSystem);
		emitter->setShape(sphere);
		EXPECT_EQ(sphere, emitter->getShape());
		EXPECT_TRUE(emitter->initialize(runtime));
		EXPECT_TRUE(emitter->wakeUp()) << "With a shape, the emitter should wakeup";
	}
}

TEST(EmitterTest, SetGetTarget)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto sphere = std::make_shared<SurgSim::Math::SphereShape>(0.1);
	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");

	{
		auto emitter = std::make_shared<Emitter>("Emitter");
		auto notParticleSystem = std::make_shared<Emitter>("Not a ParticleSystem");
		emitter->setShape(sphere);
		EXPECT_THROW(emitter->setTarget(notParticleSystem), SurgSim::Framework::AssertionFailure);
		EXPECT_TRUE(emitter->initialize(runtime));
		EXPECT_FALSE(emitter->wakeUp()) << "Without a target, the emitter should not wakup";
	}
	{
		auto emitter = std::make_shared<Emitter>("Emitter");
		emitter->setShape(sphere);
		EXPECT_NO_THROW(emitter->setTarget(particleSystem));
		EXPECT_EQ(particleSystem, emitter->getTarget());
		EXPECT_TRUE(emitter->initialize(runtime));
		EXPECT_TRUE(emitter->wakeUp()) << "With a target, the emitter should wakeup";
	}
}

TEST(EmitterTest, SetGetMode)
{
	auto emitter = std::make_shared<Emitter>("Emitter");
	EXPECT_THROW(emitter->setMode(EMIT_MODE_COUNT), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(emitter->setMode(EMIT_MODE_SURFACE));
	EXPECT_EQ(EMIT_MODE_SURFACE, emitter->getMode());
}

TEST(EmitterTest, GetSetRate)
{
	auto emitter = std::make_shared<Emitter>("Emitter");
	EXPECT_THROW(emitter->setRate(-10.0), SurgSim::Framework::AssertionFailure);
	EXPECT_NO_THROW(emitter->setRate(2.0));
	EXPECT_EQ(2.0, emitter->getRate());
}

TEST(EmitterTest, GetSetLifetimeRange)
{
	auto emitter = std::make_shared<Emitter>("Emitter");

	EXPECT_THROW(emitter->setLifetimeRange(std::make_pair(-1.0, 1.0)), SurgSim::Framework::AssertionFailure)
		<< "Negative lifetimes should not be allowed";

	EXPECT_THROW(emitter->setLifetimeRange(std::make_pair(10.0, 1.0)), SurgSim::Framework::AssertionFailure)
		<< "Should not allow lower bound to be greater than upper bound";

	EXPECT_NO_THROW(emitter->setLifetimeRange(std::make_pair(1.0, 10.0)));
	EXPECT_EQ(1.0, emitter->getLifetimeRange().first);
	EXPECT_EQ(10.0, emitter->getLifetimeRange().second);
}

TEST(EmitterTest, GetSetVelocityRange)
{
	auto emitter = std::make_shared<Emitter>("Emitter");

	EXPECT_THROW(emitter->setVelocityRange(std::make_pair(Vector3d::Constant(20.0), Vector3d::Ones())),
			SurgSim::Framework::AssertionFailure) << "Minimum velocity must be less than maximum";

	EXPECT_NO_THROW(emitter->setVelocityRange(std::make_pair(Vector3d::Ones(), Vector3d::Constant(3.0))));
	EXPECT_NO_THROW(emitter->setVelocityRange(std::make_pair(Vector3d::Constant(-2.0), Vector3d::Ones())));

	EXPECT_TRUE(emitter->getVelocityRange().first.isApprox(Vector3d::Constant(-2.0)));
	EXPECT_TRUE(emitter->getVelocityRange().second.isApprox(Vector3d::Ones()));
}

TEST(EmitterTest, Update)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto sphere = std::make_shared<SurgSim::Math::SphereShape>(0.1);

	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");
	particleSystem->setMaxParticles(10);
	particleSystem->initialize(runtime);

	auto emitter = std::make_shared<Emitter>("Emitter");
	emitter->setShape(sphere);
	emitter->setTarget(particleSystem);
	emitter->setMode(EMIT_MODE_SURFACE);
	emitter->setRate(10.0);
	emitter->setLifetimeRange(std::make_pair(5.0, 10.0));
	emitter->setVelocityRange(std::make_pair(Vector3d::Ones(), Vector3d::Constant(2.0)));
	emitter->initialize(runtime);

	ASSERT_TRUE(emitter->wakeUp());
	ASSERT_TRUE(particleSystem->wakeUp());

	emitter->update(0.1);
	EXPECT_EQ(1u, particleSystem->getParticles().unsafeGet().getNumVertices());

	emitter->update(0.05);
	EXPECT_EQ(1u, particleSystem->getParticles().unsafeGet().getNumVertices());
	emitter->update(0.05);
	EXPECT_EQ(2u, particleSystem->getParticles().unsafeGet().getNumVertices());

	emitter->update(0.9);
	EXPECT_EQ(10u, particleSystem->getParticles().unsafeGet().getNumVertices());
	emitter->update(0.2);
	EXPECT_EQ(10u, particleSystem->getParticles().unsafeGet().getNumVertices())
		<< "Particles should not have been added, the particle system should have reached its maximum.";

	particleSystem->update(1.0);
	auto particles = particleSystem->getParticles().unsafeGet().getVertices();
	ASSERT_EQ(10u, particles.size());
	for (auto particle : particles)
	{
		EXPECT_NEAR(sphere->getRadius(), particle.position.norm(), 1e-9);
		EXPECT_LT(4.0, particle.data.lifetime);
		EXPECT_GT(9.0, particle.data.lifetime);
		EXPECT_TRUE((Vector3d::Ones().array() <= particle.data.velocity.array()).all());
		EXPECT_TRUE((Vector3d::Constant(2.0).array() >= particle.data.velocity.array()).all());
	}
}

TEST(EmitterTest, PosedUpdate)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto sphere = std::make_shared<SurgSim::Math::SphereShape>(0.1);

	auto particleSystem = std::make_shared<MockParticleSystem>("ParticleSystem");
	particleSystem->setMaxParticles(10);

	auto emitter = std::make_shared<Emitter>("Emitter");
	emitter->setShape(sphere);
	emitter->setTarget(particleSystem);
	emitter->setMode(EMIT_MODE_VOLUME);
	emitter->setRate(1.0);
	emitter->setLifetimeRange(std::make_pair(5.0, 10.0));
	emitter->setVelocityRange(std::make_pair(Vector3d::Ones(), Vector3d::Constant(2.0)));

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Element");
	element->addComponent(particleSystem);
	element->addComponent(emitter);
	runtime->getScene()->addSceneElement(element);

	ASSERT_TRUE(emitter->wakeUp());
	ASSERT_TRUE(particleSystem->wakeUp());

	auto pose1 = makeRigidTransform(Eigen::AngleAxisd(0.25 * M_PI, Vector3d::UnitX()).matrix(),
			Vector3d(1.0, 2.0, 3.0));
	auto pose2 = makeRigidTransform(Eigen::AngleAxisd(0.25 * M_PI, Vector3d::UnitZ()).matrix(),
			Vector3d(2.0, -2.0, 2.0));

	emitter->update(1.0);
	EXPECT_TRUE(emitter->getPose().isApprox(Math::RigidTransform3d::Identity()));
	ASSERT_EQ(1u, particleSystem->getParticles().unsafeGet().getNumVertices());

	emitter->setLocalPose(pose1);
	emitter->update(1.0);
	EXPECT_TRUE(emitter->getPose().isApprox(pose1));
	ASSERT_EQ(2u, particleSystem->getParticles().unsafeGet().getNumVertices());

	element->setPose(pose2);
	emitter->update(1.0);
	EXPECT_TRUE(emitter->getPose().isApprox(pose2 * pose1));
	ASSERT_EQ(3u, particleSystem->getParticles().unsafeGet().getNumVertices());

	auto particles = particleSystem->getParticles().unsafeGet().getVertices();
	EXPECT_GT(sphere->getRadius(), particles[0].position.norm());
	EXPECT_GT(sphere->getRadius(), (pose1.inverse() * particles[1].position).norm());
	EXPECT_GT(sphere->getRadius(), ((pose2 * pose1).inverse() * particles[2].position).norm());
}

TEST(EmitterTest, Serialization)
{
	auto emitter = std::make_shared<Emitter>("Emitter");
	EXPECT_EQ("SurgSim::Particles::Emitter", emitter->getClassName());

	const std::pair<double, double> expectedLifetimeRange(1.0, 10.0);
	const int expectedMode = 1;
	const double expectedRate = 10.0;
	std::shared_ptr<Shape> expectedShape = std::make_shared<SurgSim::Math::SphereShape>(0.1);
	std::shared_ptr<Component> expectedTarget = std::make_shared<MockParticleSystem>("ParticleSystem");
	const std::pair<Vector3d, Vector3d> expectedVelocityRange(Vector3d::Ones(), Vector3d::Constant(2.0));
	Math::RigidTransform3d expectedPose = makeRigidTransform(Eigen::AngleAxisd(0.25 * M_PI, Vector3d::UnitX()).matrix(),
			Vector3d(1.0, 2.0, 3.0));

	EXPECT_NO_THROW(emitter->setValue("LifetimeRange", expectedLifetimeRange));
	EXPECT_NO_THROW(emitter->setValue("Mode", expectedMode));
	EXPECT_NO_THROW(emitter->setValue("Rate", expectedRate));
	EXPECT_NO_THROW(emitter->setValue("Shape", expectedShape));
	EXPECT_NO_THROW(emitter->setValue("Target", expectedTarget));
	EXPECT_NO_THROW(emitter->setValue("VelocityRange", expectedVelocityRange));
	EXPECT_NO_THROW(emitter->setValue("LocalPose", expectedPose));

	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<Component>::encode(*emitter));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(10u, node[emitter->getClassName()].size());

	std::shared_ptr<Emitter> decodedEmitter;
	ASSERT_NO_THROW(decodedEmitter = std::dynamic_pointer_cast<Emitter>(
				node.as<std::shared_ptr<Component>>()));

	auto lifetimeRange = decodedEmitter->getValue<std::pair<double, double>>("LifetimeRange");
	EXPECT_EQ(expectedLifetimeRange, lifetimeRange);
	EXPECT_EQ(expectedMode, decodedEmitter->getValue<int>("Mode"));
	EXPECT_EQ(expectedRate, decodedEmitter->getValue<double>("Rate"));
	EXPECT_EQ(expectedShape->getType(), decodedEmitter->getValue<std::shared_ptr<Shape>>("Shape")->getType());
	EXPECT_EQ("ParticleSystem", decodedEmitter->getValue<std::shared_ptr<Component>>("Target")->getName());
	auto velocityRange = decodedEmitter->getValue<std::pair<Vector3d, Vector3d>>("VelocityRange");
	EXPECT_TRUE(expectedVelocityRange.first.isApprox(velocityRange.first));
	EXPECT_TRUE(expectedVelocityRange.second.isApprox(velocityRange.second));
	EXPECT_TRUE(expectedPose.isApprox(decodedEmitter->getValue<Math::RigidTransform3d>("LocalPose")));
}

}; // namespace Particles
}; // namespace SurgSim
