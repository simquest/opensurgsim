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

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/ParticlesState.h"
#include "SurgSim/Particles/ParticleSystemRepresentation.h"

using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Particles
{

TEST(ParticleReferenceTests, CanConstruct)
{
	auto state = std::make_shared<ParticlesState>();
	ASSERT_NO_THROW(ParticleReference reference(state, 0););
}

TEST(ParticleReferenceTests, GetSetPosition)
{
	auto state = std::make_shared<ParticlesState>();
	state->setNumDof(3u, 2u);
	ParticleReference reference(state, 1);

	EXPECT_TRUE(reference.getPosition().isZero());
	reference.setPosition(Vector3d::Constant(2.0));
	EXPECT_TRUE(Vector3d::Constant(2.0).isApprox(reference.getPosition()));
	EXPECT_TRUE(state->getPositions().isApprox((Vector(6) << 0.0, 0.0, 0.0, 2.0, 2.0, 2.0).finished()));
	EXPECT_TRUE(state->getVelocities().isZero());
	EXPECT_TRUE(state->getLifetimes().isZero());
}

TEST(ParticleReferenceTests, GetSetVelocity)
{
	auto state = std::make_shared<ParticlesState>();
	state->setNumDof(3u, 2u);
	ParticleReference reference(state, 1);

	EXPECT_TRUE(reference.getVelocity().isZero());
	reference.setVelocity(Vector3d::Constant(5.0));
	EXPECT_TRUE(Vector3d::Constant(5.0).isApprox(reference.getVelocity()));
	EXPECT_TRUE(state->getPositions().isZero());
	EXPECT_TRUE(state->getVelocities().isApprox((Vector(6) << 0.0, 0.0, 0.0, 5.0, 5.0, 5.0).finished()));
	EXPECT_TRUE(state->getLifetimes().isZero());
}

TEST(ParticleReferenceTests, GetSetLifetime)
{
	auto state = std::make_shared<ParticlesState>();
	state->setNumDof(3u, 2u);
	ParticleReference reference(state, 1);

	EXPECT_NEAR(0.0, reference.getLifetime(), 1e-9);
	reference.setLifetime(20.0);
	EXPECT_NEAR(20.0, reference.getLifetime(), 1e-9);
	EXPECT_TRUE(state->getPositions().isZero());
	EXPECT_TRUE(state->getVelocities().isZero());
	EXPECT_TRUE(state->getLifetimes().isApprox((Vector(2) << 0.0, 20.0).finished()));
}

TEST(ParticleReferenceTests, CanAssign)
{
	auto state = std::make_shared<ParticlesState>();
	state->setNumDof(3u, 5u);

	ParticleReference reference(state, 0);
	Particle particle(Vector3d(1.0, 2.0, 3.0), Vector3d(4.0, 5.0, 6.0), 7.0);
	reference = particle;

	EXPECT_TRUE(particle.getPosition().isApprox(reference.getPosition()));
	EXPECT_TRUE(particle.getVelocity().isApprox(reference.getVelocity()));
	EXPECT_NEAR(particle.getLifetime(), reference.getLifetime(), 1e-9);
}

}; // namespace Particles
}; // namespace SurgSim
