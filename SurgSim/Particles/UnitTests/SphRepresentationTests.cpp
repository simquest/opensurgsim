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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/SphRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Particles
{

TEST(SphRepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW(std::make_shared<SphRepresentation>("representation"));
}

TEST(SphRepresentationTest, SetGetTest)
{
	auto sph = std::make_shared<SphRepresentation>("representation");

	EXPECT_DOUBLE_EQ(0.0, sph->getMassPerParticle());
	EXPECT_THROW(sph->setMassPerParticle(0.0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(sph->setMassPerParticle(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setMassPerParticle(0.02);
	EXPECT_DOUBLE_EQ(0.02, sph->getMassPerParticle());

	EXPECT_DOUBLE_EQ(0.0, sph->getDensity());
	EXPECT_THROW(sph->setDensity(0.0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(sph->setDensity(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setDensity(0.03);
	EXPECT_DOUBLE_EQ(0.03, sph->getDensity());

	EXPECT_DOUBLE_EQ(0.0, sph->getGasStiffness());
	EXPECT_THROW(sph->setGasStiffness(0.0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(sph->setGasStiffness(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setGasStiffness(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getGasStiffness());

	EXPECT_DOUBLE_EQ(0.0, sph->getSurfaceTension());
	EXPECT_NO_THROW(sph->setSurfaceTension(0.0));
	EXPECT_THROW(sph->setSurfaceTension(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setSurfaceTension(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getSurfaceTension());

	EXPECT_TRUE(sph->getGravity().isApprox(SurgSim::Math::Vector3d(0.0, -9.81, 0.0)));
	sph->setGravity(SurgSim::Math::Vector3d::Ones() * 0.56);
	EXPECT_TRUE(sph->getGravity().isApprox(SurgSim::Math::Vector3d::Ones() * 0.56));

	EXPECT_DOUBLE_EQ(0.0, sph->getViscosity());
	EXPECT_NO_THROW(sph->setViscosity(0.0));
	EXPECT_THROW(sph->setViscosity(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setViscosity(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getViscosity());

	EXPECT_DOUBLE_EQ(0.0, sph->getStiffness());
	EXPECT_NO_THROW(sph->setStiffness(0.0));
	sph->setStiffness(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getStiffness());

	EXPECT_DOUBLE_EQ(0.0, sph->getDamping());
	EXPECT_NO_THROW(sph->setDamping(0.0));
	sph->setDamping(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getDamping());

	EXPECT_DOUBLE_EQ(0.0, sph->getFriction());
	EXPECT_NO_THROW(sph->setFriction(0.0));
	sph->setFriction(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getFriction());

	EXPECT_DOUBLE_EQ(0.0, sph->getKernelSupport());
	EXPECT_THROW(sph->setKernelSupport(0.0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(sph->setKernelSupport(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setKernelSupport(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getKernelSupport());
}

TEST(SphRepresentationTest, DoInitializeTest)
{
	{
		SCOPED_TRACE("Bad Mass Per Particle");

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto sph = std::make_shared<SphRepresentation>("representation");
		EXPECT_THROW(sph->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Bad density reference");

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto sph = std::make_shared<SphRepresentation>("representation");
		sph->setMassPerParticle(0.02);
		EXPECT_THROW(sph->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Bad gas stiffness");

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto sph = std::make_shared<SphRepresentation>("representation");
		sph->setMassPerParticle(0.02);
		sph->setDensity(0.02);
		EXPECT_THROW(sph->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("Bad kernel support");

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto sph = std::make_shared<SphRepresentation>("representation");
		sph->setMassPerParticle(0.02);
		sph->setDensity(0.02);
		sph->setGasStiffness(0.02);
		EXPECT_THROW(sph->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}

	{
		SCOPED_TRACE("All set");

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
		auto sph = std::make_shared<SphRepresentation>("representation");
		sph->setMassPerParticle(0.02);
		sph->setDensity(0.02);
		sph->setGasStiffness(0.02);
		sph->setKernelSupport(0.02);
		EXPECT_NO_THROW(sph->initialize(runtime));
	}
}

TEST(SphRepresentationTest, DoUpdate1ParticleTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto sph = std::make_shared<SphRepresentation>("representation");
	double dt = 1e-3; // 1ms

	sph->setMaxParticles(1);
	sph->setMassPerParticle(0.02);
	sph->setDensity(1000.0);
	sph->setGasStiffness(3.0);
	sph->setKernelSupport(0.01);
	sph->setViscosity(0.01);
	sph->setSurfaceTension(0.0);

	sph->initialize(runtime);

	sph->addParticle(Math::Vector3d::Zero(), Math::Vector3d::Zero(), 10);
	EXPECT_EQ(1u, sph->getParticles().unsafeGet().getNumVertices());

	EXPECT_NO_THROW(sph->update(dt));
	auto particles = sph->getParticles().safeGet()->getVertices();
	EXPECT_EQ(1u, particles.size());
	EXPECT_DOUBLE_EQ(0.0, particles[0].position[0]);
	EXPECT_GT(0.0, particles[0].position[1]);
	EXPECT_DOUBLE_EQ(0.0, particles[0].position[2]);

	EXPECT_DOUBLE_EQ(0.0, particles[0].data.velocity[0]);
	EXPECT_GT(0.0, particles[0].data.velocity[1]);
	EXPECT_DOUBLE_EQ(0.0, particles[0].data.velocity[2]);
}

namespace
{
/// Set up a Unit Test where 2 particles interact with a radius R (h/2) of null interaction
/// If they are closer than R, they repel each other
/// If they are at distance R from each other, they are in equilibrium
/// If they are further than R, the attract each other
std::shared_ptr<SphRepresentation> set2ParticlesInteracting(double h, double distance)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto sph = std::make_shared<SphRepresentation>("representation");
	double dt = 1e-3; // 1ms

	sph->setMaxParticles(2);
	sph->setMassPerParticle(0.02);	// 50 particles for 0.001 m3 at 1000Kg.m-3
	// If you note R the radius of 1 particle, then (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	sph->setDensity(1000.0);
	sph->setGasStiffness(3.0);
	sph->setKernelSupport(h);
	sph->setViscosity(0.01);
	sph->setSurfaceTension(0.0);

	sph->initialize(runtime);

	sph->addParticle(Math::Vector3d::Zero(), Math::Vector3d::Zero(), 10);
	sph->addParticle(Math::Vector3d(distance, 0.0, 0.0), Math::Vector3d::Zero(), 10);
	EXPECT_EQ(2u, sph->getParticles().unsafeGet().getNumVertices());

	EXPECT_NO_THROW(sph->update(dt));

	EXPECT_EQ(2u, sph->getParticles().unsafeGet().getNumVertices());
	size_t index = 0;
	for (auto particle : sph->getParticles().unsafeGet().getVertices())
	{
		std::string scope = "Particle "+boost::to_string(index);
		SCOPED_TRACE(scope);
		EXPECT_LT(particle.position[1], 0.0);
		EXPECT_DOUBLE_EQ(0.0, particle.position[2]);

		EXPECT_GT(0.0, particle.data.velocity[1]);
		EXPECT_DOUBLE_EQ(0.0, particle.data.velocity[2]);
		index++;
	}

	return sph;
}
}; // namespace anonymous

TEST(SphRepresentationTest, DoUpdate2ParticlesNotInteractingTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = 10.0 * h; // Far from their radius of influence
	auto sph = set2ParticlesInteracting(h, distance);

	auto& particles = sph->getParticles().unsafeGet().getVertices();
	EXPECT_DOUBLE_EQ(particles[0].position[1], particles[1].position[1]);
	EXPECT_DOUBLE_EQ(particles[0].data.velocity[1], particles[1].data.velocity[1]);
	double finalDistance = (particles[0].position - particles[1].position).norm();
	EXPECT_DOUBLE_EQ(distance, finalDistance);
}

TEST(SphRepresentationTest, DoUpdate2ParticlesAttractingTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = h * 3.0 / 4.0;
	auto sph = set2ParticlesInteracting(h, distance);

	auto& particles = sph->getParticles().unsafeGet().getVertices();
	EXPECT_DOUBLE_EQ(particles[0].position[1], particles[1].position[1]);
	EXPECT_DOUBLE_EQ(particles[0].data.velocity[1], particles[1].data.velocity[1]);
	double finalDistance = (particles[0].position - particles[1].position).norm();
	EXPECT_GT(distance, finalDistance);
}

TEST(SphRepresentationTest, DoUpdate2ParticlesRetractingTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = h * 1.0 / 4.0;
	auto sph = set2ParticlesInteracting(h, distance);

	auto& particles = sph->getParticles().unsafeGet().getVertices();
	EXPECT_DOUBLE_EQ(particles[0].position[1], particles[1].position[1]);
	EXPECT_DOUBLE_EQ(particles[0].data.velocity[1], particles[1].data.velocity[1]);
	double finalDistance = (particles[0].position - particles[1].position).norm();
	EXPECT_LT(distance, finalDistance);
}

TEST(SphRepresentationTest, DoUpdate2ParticlesInEquilibriumTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = h / 2.0;
	auto sph = set2ParticlesInteracting(h, distance);

	auto& particles = sph->getParticles().unsafeGet().getVertices();
	EXPECT_DOUBLE_EQ(particles[0].position[1], particles[1].position[1]);
	EXPECT_DOUBLE_EQ(particles[0].data.velocity[1], particles[1].data.velocity[1]);
	double finalDistance = (particles[0].position - particles[1].position).norm();
	EXPECT_NEAR(distance, finalDistance, pow(h, 2));
}

TEST(SphRepresentationTest, SerializationTest)
{
	auto sph = std::make_shared<SphRepresentation>("TestSphRepresentation");
	sph->setDensity(1.1);
	sph->setGasStiffness(2.2);
	sph->setGravity(SurgSim::Math::Vector3d::Ones());
	sph->setKernelSupport(3.3);
	sph->setMassPerParticle(4.4);
	sph->setMaxParticles(5);
	sph->setSurfaceTension(10.1);
	sph->setViscosity(11.11);
	sph->setStiffness(12.12);
	sph->setDamping(13.13);
	sph->setFriction(0.14);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*sph));

	std::shared_ptr<SphRepresentation> newRepresentation;
	EXPECT_NO_THROW(newRepresentation =
		std::dynamic_pointer_cast<SphRepresentation>(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_DOUBLE_EQ(sph->getDensity(), newRepresentation->getValue<double>("Density"));
	EXPECT_DOUBLE_EQ(sph->getGasStiffness(), newRepresentation->getValue<double>("GasStiffness"));
	EXPECT_TRUE(sph->getGravity().isApprox(newRepresentation->getValue<SurgSim::Math::Vector3d>("Gravity")));
	EXPECT_DOUBLE_EQ(sph->getKernelSupport(), newRepresentation->getValue<double>("KernelSupport"));
	EXPECT_DOUBLE_EQ(sph->getMassPerParticle(), newRepresentation->getValue<double>("MassPerParticle"));
	EXPECT_EQ(sph->getMaxParticles(), newRepresentation->getValue<size_t>("MaxParticles"));
	EXPECT_DOUBLE_EQ(sph->getSurfaceTension(), newRepresentation->getValue<double>("SurfaceTension"));
	EXPECT_DOUBLE_EQ(sph->getViscosity(), newRepresentation->getValue<double>("Viscosity"));
	EXPECT_DOUBLE_EQ(sph->getStiffness(), newRepresentation->getValue<double>("Stiffness"));
	EXPECT_DOUBLE_EQ(sph->getDamping(), newRepresentation->getValue<double>("Damping"));
	EXPECT_DOUBLE_EQ(sph->getFriction(), newRepresentation->getValue<double>("Friction"));
}

}; // namespace Particles
}; // namespace SurgSim
