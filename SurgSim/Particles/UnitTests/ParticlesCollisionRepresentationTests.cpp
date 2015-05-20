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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/ParticlesShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"
#include "SurgSim/Particles/Representation.h"
#include "SurgSim/Particles/UnitTests/MockObjects.h"

namespace
{
const double epsilon = 1e-10;
}

namespace SurgSim
{
namespace Particles
{

struct ParticlesCollisionRepresentationTest : public ::testing::Test
{
	void SetUp()
	{
		m_runtime = std::make_shared<SurgSim::Framework::Runtime>();
		m_particleRepresentation = std::make_shared<MockParticleSystem>("ParticleRepresentation");
		m_particleCollisionRepresentation =
			std::make_shared<ParticlesCollisionRepresentation>("ParticlesCollisionRepresentation");
	}

	std::shared_ptr<SurgSim::Framework::Runtime> m_runtime;
	std::shared_ptr<SurgSim::Particles::Representation> m_particleRepresentation;
	std::shared_ptr<SurgSim::Particles::ParticlesCollisionRepresentation> m_particleCollisionRepresentation;
};

TEST_F(ParticlesCollisionRepresentationTest, InitTest)
{
	EXPECT_NO_THROW(ParticlesCollisionRepresentation("TestParticlesCollisionRepresentation"));
}

TEST_F(ParticlesCollisionRepresentationTest, SetGetParticlesRepresentationTest)
{
	EXPECT_THROW(m_particleCollisionRepresentation->getParticleRepresentation(), SurgSim::Framework::AssertionFailure);
	ASSERT_NO_THROW(m_particleCollisionRepresentation->setParticleRepresentation(m_particleRepresentation));
	EXPECT_EQ(m_particleRepresentation, m_particleCollisionRepresentation->getParticleRepresentation());
}

TEST_F(ParticlesCollisionRepresentationTest, SetGetParticleRadius)
{
	ASSERT_NO_THROW(m_particleCollisionRepresentation->setParticleRadius(4.0));
	EXPECT_EQ(4.0, m_particleCollisionRepresentation->getParticleRadius());
}

TEST_F(ParticlesCollisionRepresentationTest, ShapeTypeTest)
{
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_PARTICLES, m_particleCollisionRepresentation->getShapeType());
}

TEST_F(ParticlesCollisionRepresentationTest, SerializationTest)
{
	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*m_particleCollisionRepresentation));

	std::shared_ptr<SurgSim::Particles::ParticlesCollisionRepresentation> newParticlesCollisionRepresentation;
	ASSERT_NO_THROW(newParticlesCollisionRepresentation =
						std::dynamic_pointer_cast<SurgSim::Particles::ParticlesCollisionRepresentation>
						(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
}

TEST_F(ParticlesCollisionRepresentationTest, UpdateAndInitializationTest)
{
	m_particleRepresentation->setCollisionRepresentation(m_particleCollisionRepresentation);
	m_particleRepresentation->setMaxParticles(100);
	m_particleRepresentation->addParticle(Math::Vector3d::Zero(), Math::Vector3d::Zero(), 1.5);
	ASSERT_TRUE(m_particleRepresentation->initialize(m_runtime));
	ASSERT_TRUE(m_particleRepresentation->wakeUp());

	auto shape = std::dynamic_pointer_cast<Math::ParticlesShape>(m_particleCollisionRepresentation->getShape());
	EXPECT_EQ(0u, shape->getNumVertices());

	EXPECT_TRUE(m_particleCollisionRepresentation->initialize(m_runtime));
	EXPECT_TRUE(m_particleCollisionRepresentation->wakeUp());

	EXPECT_NO_THROW(m_particleRepresentation->update(1.0));
	EXPECT_NO_THROW(m_particleCollisionRepresentation->update(1.0));
	EXPECT_EQ(1u, shape->getNumVertices());

	EXPECT_NO_THROW(m_particleRepresentation->update(1.0));
	EXPECT_NO_THROW(m_particleCollisionRepresentation->update(1.0));
	EXPECT_EQ(0u, shape->getNumVertices());
}

};
};
