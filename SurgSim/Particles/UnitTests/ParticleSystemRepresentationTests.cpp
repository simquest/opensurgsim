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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/ParticlesState.h"
#include "SurgSim/Particles/ParticleSystemRepresentation.h"
#include "SurgSim/Particles/UnitTests/MockObjects.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Particles
{

TEST(ParticleSystemRepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW(MockParticleSystem representation("representation"));

	MockParticleSystem representation("representation");
	EXPECT_EQ(0u, representation.getMaxParticles());
}

TEST(ParticleSystemRepresentationTest, SetGetMaxParticles)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();

	EXPECT_NO_THROW(representation->setMaxParticles(1));
	EXPECT_EQ(1u, representation->getMaxParticles());

	representation->initialize(runtime);
	EXPECT_ANY_THROW(representation->setMaxParticles(1));

	EXPECT_EQ(0, representation->getParticleReferences().size());
}

TEST(ParticleSystemRepresentationTest, AddParticle)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(1);
	representation->initialize(runtime);

	Particle expectedParticle(Particle(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 10));
	ASSERT_TRUE(representation->addParticle(expectedParticle));
	ASSERT_EQ(1, representation->getParticleReferences().size());

	auto particle = representation->getParticleReferences().begin();
	EXPECT_TRUE(expectedParticle.getPosition().isApprox(particle->getPosition()));
	EXPECT_TRUE(expectedParticle.getVelocity().isApprox(particle->getVelocity()));
	EXPECT_NEAR(expectedParticle.getLifetime(), particle->getLifetime(), 1e-9);
}

TEST(ParticleSystemRepresentationTest, AddParticles)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(100);
	representation->initialize(runtime);

	std::vector<Particle> expectedParticles;
	expectedParticles.emplace_back(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 10);
	expectedParticles.emplace_back(Vector3d(2.0, 2.0, 2.0), Vector3d(0.0, 0.0, 0.0), 20);
	expectedParticles.emplace_back(Vector3d(3.0, 3.0, 3.0), Vector3d(0.0, 0.0, 0.0), 30);

	ASSERT_TRUE(representation->addParticles(expectedParticles));
	ASSERT_EQ(3, representation->getParticleReferences().size());

	auto expectedParticle = expectedParticles.cbegin();
	auto particle = representation->getParticleReferences().begin();
	for( ; expectedParticle != expectedParticles.cend(); ++expectedParticle, ++particle)
	{
		EXPECT_TRUE(expectedParticle->getPosition().isApprox(particle->getPosition()));
		EXPECT_TRUE(expectedParticle->getVelocity().isApprox(particle->getVelocity()));
		EXPECT_NEAR(expectedParticle->getLifetime(), particle->getLifetime(), 1e-9);
	}
}

TEST(ParticleSystemRepresentationTest, GetParticles)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(100);
	representation->initialize(runtime);

	Particle expectedParticle(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 10);
	representation->addParticle(expectedParticle);
	{
		EXPECT_EQ(0, representation->getParticles()->size())
			<< "Added particle should not be yet avaible in safe access";
		EXPECT_EQ(1, representation->getParticleReferences().size())
			<< "Added particle should be available in unsafe access";
	}

	representation->update(1.0);
	{
		ASSERT_EQ(1, representation->getParticles()->size())
			<< "Added particle should be avaible in safe access after update";
		EXPECT_EQ(1, representation->getParticleReferences().size())
			<< "Added particle should be available in unsafe access";

		auto particles = representation->getParticles();
		EXPECT_TRUE(expectedParticle.getPosition().isApprox((*particles)[0].getPosition()));
		EXPECT_TRUE(expectedParticle.getVelocity().isApprox((*particles)[0].getVelocity()));
		EXPECT_NEAR(expectedParticle.getLifetime() - 1.0, (*particles)[0].getLifetime(), 1e-9);
	}

	representation->removeParticle(*representation->getParticleReferences().begin());
	{
		EXPECT_EQ(1, representation->getParticles()->size())
			<< "Removed particle should still be avaible in safe access";
		EXPECT_EQ(0, representation->getParticleReferences().size())
			<< "Removed particle should already be removed in unsafe access";
	}

	representation->update(1.0);
	{
		EXPECT_EQ(0, representation->getParticles()->size())
			<< "Removed particle should be removed in safe access";
		EXPECT_EQ(0, representation->getParticleReferences().size())
			<< "Removed particle should be removed in unsafe access";
	}
}

TEST(ParticleSystemRepresentationTest, RemoveParticle)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(10);
	representation->initialize(runtime);

	ASSERT_TRUE(representation->addParticle(Particle(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 10)));
	ASSERT_EQ(1, representation->getParticleReferences().size());

	ParticleReference particle = *(representation->getParticleReferences().begin());
	EXPECT_TRUE(representation->removeParticle(particle));
	EXPECT_EQ(0, representation->getParticleReferences().size());

	EXPECT_FALSE(representation->removeParticle(particle));
}

TEST(ParticleSystemRepresentationTest, MaxParticles)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	std::vector<Particle> particles;
	particles.emplace_back(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 10);
	particles.emplace_back(Vector3d(2.0, 2.0, 2.0), Vector3d(0.0, 0.0, 0.0), 20);
	particles.emplace_back(Vector3d(3.0, 3.0, 3.0), Vector3d(0.0, 0.0, 0.0), 30);

	{
		auto representation = std::make_shared<MockParticleSystem>("representation");
		representation->setMaxParticles(1);
		representation->initialize(runtime);
		EXPECT_TRUE(representation->addParticle(particles[0]));
		EXPECT_FALSE(representation->addParticle(particles[1]));
	}

	{
		auto representation = std::make_shared<MockParticleSystem>("representation");
		representation->setMaxParticles(4);
		representation->initialize(runtime);
		EXPECT_TRUE(representation->addParticles(particles));
		EXPECT_FALSE(representation->addParticles(particles));
	}
}

TEST(ParticleSystemRepresentationTest, Aging)
{
	auto representation = std::make_shared<MockParticleSystem>("representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	representation->setMaxParticles(100);
	representation->initialize(runtime);

	representation->addParticle(Particle(Vector3d(1.0, 1.0, 1.0), Vector3d(0.0, 0.0, 0.0), 1));
	representation->addParticle(Particle(Vector3d(2.0, 2.0, 2.0), Vector3d(0.0, 0.0, 0.0), 2));
	representation->addParticle(Particle(Vector3d(3.0, 3.0, 3.0), Vector3d(0.0, 0.0, 0.0), 3));
	EXPECT_EQ(3, representation->getParticleReferences().size());

	representation->update(1.0);
	EXPECT_EQ(2, representation->getParticleReferences().size());

	representation->update(1.0);
	EXPECT_EQ(1, representation->getParticleReferences().size());

	representation->update(1.0);
	EXPECT_EQ(0, representation->getParticleReferences().size());

	representation->update(1.0);
	EXPECT_EQ(0, representation->getParticleReferences().size());
}

}; // namespace Particles
}; // namespace SurgSim
