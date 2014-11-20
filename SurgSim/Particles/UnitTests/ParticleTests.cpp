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

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Particle.h"

using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Particles
{

TEST(ParticleTests, InitTest)
{
	ASSERT_NO_THROW(Particle particle;);

	{
		Particle particle;
		EXPECT_TRUE(particle.getPosition().isZero());
		EXPECT_TRUE(particle.getVelocity().isZero());
		EXPECT_NEAR(0.0, particle.getLifetime(), 1e-9);
	}

	{
		Particle particle(Vector3d(1.0, 1.0, 1.0), Vector3d(2.0, 2.0, 2.0), 3.0);
		EXPECT_TRUE(Vector3d(1.0, 1.0, 1.0).isApprox(particle.getPosition()));
		EXPECT_TRUE(Vector3d(2.0, 2.0, 2.0).isApprox(particle.getVelocity()));
		EXPECT_NEAR(3.0, particle.getLifetime(), 1e-9);
	}
}

TEST(ParticleTests, GetSetTest)
{
	Particle particle;
	particle.setPosition(Vector3d(1.0, 2.0, 3.0));
	particle.setVelocity(Vector3d(4.0, 5.0, 6.0));
	particle.setLifetime(7.0);

	EXPECT_TRUE(Vector3d(1.0, 2.0, 3.0).isApprox(particle.getPosition()));
	EXPECT_TRUE(Vector3d(4.0, 5.0, 6.0).isApprox(particle.getVelocity()));
	EXPECT_NEAR(7.0, particle.getLifetime(), 1e-9);
}

}; // namespace Particles
}; // namespace SurgSim
