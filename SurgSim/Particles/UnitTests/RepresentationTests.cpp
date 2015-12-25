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

#include <gtest/gtest.h>
#include <memory>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Emitter.h"
#include "SurgSim/Particles/Representation.h"
#include "SurgSim/Particles/UnitTests/MockObjects.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Particles
{

TEST(RepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW(MockParticleSystem representation("representation"));

	MockParticleSystem representation("representation");
	EXPECT_EQ(0u, representation.getMaxParticles());
}

TEST(RepresentationTest, SetGetMaxParticles)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();

	EXPECT_NO_THROW(representation->setMaxParticles(1));
	EXPECT_EQ(1u, representation->getMaxParticles());
}

TEST(RepresentationTest, AddParticle)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(1);
	representation->initialize(runtime);

	ASSERT_TRUE(representation->addParticle(Vector3d::Ones(), Vector3d::Constant(3.0), 10));
	ASSERT_EQ(1, representation->getParticles().unsafeGet().getNumVertices());

	auto& particle = representation->getParticles().unsafeGet().getVertices()[0];
	EXPECT_TRUE(particle.position.isApprox(Vector3d::Ones()));
	EXPECT_TRUE(particle.data.velocity.isApprox(Vector3d::Constant(3.0)));
	EXPECT_NEAR(10.0, particle.data.lifetime, 1e-9);
}

TEST(RepresentationTest, RemoveParticle)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(2);
	representation->initialize(runtime);

	ASSERT_TRUE(representation->addParticle(Vector3d::Ones(), Vector3d::Constant(3.0), 10));
	ASSERT_TRUE(representation->addParticle(Vector3d(2.0, 3.0, 4.0), Vector3d::Zero(), 10));
	ASSERT_EQ(2, representation->getParticles().unsafeGet().getNumVertices());
	representation->update(1.0);
	ASSERT_EQ(2, representation->getParticles().safeGet()->getNumVertices());

	representation->removeParticle(0);
	representation->update(1.0);
	auto& particleVertices = representation->getParticles().safeGet()->getVertices();
	ASSERT_EQ(1, particleVertices.size());
	EXPECT_TRUE(particleVertices[0].position.isApprox(Vector3d(2.0, 3.0, 4.0)));
	EXPECT_TRUE(particleVertices[0].data.velocity.isApprox(Vector3d::Zero()));

	representation->removeParticle(0);
	representation->update(1.0);
	ASSERT_EQ(0, representation->getParticles().safeGet()->getNumVertices());
}

TEST(RepresentationTest, GetParticles)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	representation->setMaxParticles(100);

	Vector3d expectedPosition = Vector3d(1.0, 2.0, 3.0);
	Vector3d expectedVelocity = Vector3d(-4.0, 5.0, -6.0);
	double expectedLifetime = 12.34;
	representation->addParticle(expectedPosition, expectedVelocity, expectedLifetime);
	ASSERT_EQ(1, representation->getParticles().unsafeGet().getNumVertices());

	auto& particles = representation->getParticles().unsafeGet().getVertices();
	EXPECT_TRUE(expectedPosition.isApprox(particles[0].position));
	EXPECT_TRUE(expectedVelocity.isApprox(particles[0].data.velocity));
	EXPECT_NEAR(expectedLifetime, particles[0].data.lifetime, 1e-9);
}

TEST(RepresentationTest, MaxParticles)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	representation->setMaxParticles(1);
	EXPECT_TRUE(representation->addParticle(Vector3d::Constant(1.0), Vector3d::Zero(), 10));
	EXPECT_FALSE(representation->addParticle(Vector3d::Constant(2.0), Vector3d::Zero(), 10));
}

TEST(RepresentationTest, Aging)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(100);
	representation->initialize(runtime);

	representation->addParticle(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 1);
	representation->addParticle(Vector3d(2.0, 2.0, 2.0), Vector3d(0.0, 0.0, 0.0), 2);
	representation->addParticle(Vector3d(3.0, 3.0, 3.0), Vector3d(0.0, 0.0, 0.0), 3);
	EXPECT_EQ(3, representation->getParticles().unsafeGet().getNumVertices());

	representation->update(1.0);
	EXPECT_EQ(2, representation->getParticles().unsafeGet().getNumVertices());
	EXPECT_EQ(2, representation->getParticles().safeGet()->getNumVertices());

	representation->update(1.0);
	EXPECT_EQ(1, representation->getParticles().unsafeGet().getNumVertices());
	EXPECT_EQ(1, representation->getParticles().safeGet()->getNumVertices());

	representation->update(1.0);
	EXPECT_EQ(0, representation->getParticles().unsafeGet().getNumVertices());
	EXPECT_EQ(0, representation->getParticles().safeGet()->getNumVertices());

	representation->update(1.0);
	EXPECT_EQ(0, representation->getParticles().unsafeGet().getNumVertices());
	EXPECT_EQ(0, representation->getParticles().safeGet()->getNumVertices());
}

}; // namespace Particles
}; // namespace SurgSim
