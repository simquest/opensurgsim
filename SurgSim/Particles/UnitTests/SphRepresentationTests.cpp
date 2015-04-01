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

	EXPECT_DOUBLE_EQ(0.0, sph->getKernelSupport());
	EXPECT_THROW(sph->setKernelSupport(0.0), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(sph->setKernelSupport(-1.0), SurgSim::Framework::AssertionFailure);
	sph->setKernelSupport(0.04);
	EXPECT_DOUBLE_EQ(0.04, sph->getKernelSupport());
}

TEST(SphRepresentationTest, AddGetConstraintPlaneTest)
{
	auto sph = std::make_shared<SphRepresentation>("representation");

	EXPECT_EQ(0u, sph->getPlaneConstraints().size());

	SphRepresentation::PlaneConstraint p1;
	p1.damping = 0.5;
	p1.stiffness = 0.6;
	p1.planeEquation = SurgSim::Math::Vector4d(0.1, 0.2, 0.3, 0.4);
	EXPECT_NO_THROW(sph->addPlaneConstraint(p1));
	EXPECT_EQ(1u, sph->getPlaneConstraints().size());
	EXPECT_DOUBLE_EQ(0.5, sph->getPlaneConstraints()[0].damping);
	EXPECT_DOUBLE_EQ(0.6, sph->getPlaneConstraints()[0].stiffness);
	EXPECT_TRUE(sph->getPlaneConstraints()[0].planeEquation.isApprox(SurgSim::Math::Vector4d(0.1, 0.2, 0.3, 0.4)));

	SphRepresentation::PlaneConstraint p2;
	p2.damping = 0.55;
	p2.stiffness = 0.66;
	p2.planeEquation = SurgSim::Math::Vector4d(0.11, 0.22, 0.33, 0.44);
	EXPECT_NO_THROW(sph->addPlaneConstraint(p2));
	EXPECT_EQ(2u, sph->getPlaneConstraints().size());
	EXPECT_DOUBLE_EQ(0.5, sph->getPlaneConstraints()[0].damping);
	EXPECT_DOUBLE_EQ(0.6, sph->getPlaneConstraints()[0].stiffness);
	EXPECT_TRUE(sph->getPlaneConstraints()[0].planeEquation.isApprox(SurgSim::Math::Vector4d(0.1, 0.2, 0.3, 0.4)));
	EXPECT_DOUBLE_EQ(0.55, sph->getPlaneConstraints()[1].damping);
	EXPECT_DOUBLE_EQ(0.66, sph->getPlaneConstraints()[1].stiffness);
	EXPECT_TRUE(sph->getPlaneConstraints()[1].planeEquation.isApprox(SurgSim::Math::Vector4d(0.11, 0.22, 0.33, 0.44)));
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

	Particle p;
	p.setLifetime(10);
	p.setPosition(SurgSim::Math::Vector3d::Zero());
	p.setVelocity(SurgSim::Math::Vector3d::Zero());
	sph->addParticle(p); // Add 1 particle in the sph system
	EXPECT_EQ(1u, sph->getParticleReferences().size());

	EXPECT_NO_THROW(sph->update(dt));
	EXPECT_EQ(1u, sph->getParticleReferences().size());
	EXPECT_DOUBLE_EQ(0.0, sph->getParticleReferences().front().getPosition()[0]);
	EXPECT_LT(sph->getParticleReferences().front().getPosition()[1], 0.0);
	EXPECT_DOUBLE_EQ(0.0, sph->getParticleReferences().front().getPosition()[2]);

	EXPECT_DOUBLE_EQ(0.0, sph->getParticleReferences().front().getVelocity()[0]);
	EXPECT_LT(sph->getParticleReferences().front().getVelocity()[1], 0.0);
	EXPECT_DOUBLE_EQ(0.0, sph->getParticleReferences().front().getVelocity()[2]);
}

TEST(SphRepresentationTest, ParticleOutsideGridTest)
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

	Particle p;
	p.setLifetime(10);
	p.setPosition(SurgSim::Math::Vector3d::Ones() * 1e10);
	p.setVelocity(SurgSim::Math::Vector3d::Zero());
	sph->addParticle(p); // Add 1 particle in the sph system
	EXPECT_EQ(1u, sph->getParticleReferences().size());

	EXPECT_NO_THROW(sph->update(dt));
	EXPECT_EQ(1u, sph->getParticleReferences().size());
	EXPECT_NO_THROW(sph->update(dt));
	EXPECT_EQ(0u, sph->getParticleReferences().size());
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

	Particle p;
	p.setLifetime(10);
	p.setPosition(SurgSim::Math::Vector3d::Zero());
	p.setVelocity(SurgSim::Math::Vector3d::Zero());
	sph->addParticle(p); // Add 1 particle in the sph system
	p.setPosition(SurgSim::Math::Vector3d(distance, 0.0, 0.0));
	sph->addParticle(p); // Add 1 particle in the sph system
	EXPECT_EQ(2u, sph->getParticleReferences().size());

	EXPECT_NO_THROW(sph->update(dt));

	EXPECT_EQ(2u, sph->getParticleReferences().size());
	for (auto particle : sph->getParticleReferences())
	{
		std::string scope = "Particle "+boost::to_string(particle.getIndex());
		SCOPED_TRACE(scope);
		EXPECT_LT(particle.getPosition()[1], 0.0);
		EXPECT_DOUBLE_EQ(0.0, particle.getPosition()[2]);

		EXPECT_LT(particle.getVelocity()[1], 0.0);
		EXPECT_DOUBLE_EQ(0.0, particle.getVelocity()[2]);
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

	auto x0 = sph->getParticleReferences().front().getPosition();
	auto x1 = sph->getParticleReferences().back().getPosition();
	auto v0 = sph->getParticleReferences().front().getPosition();
	auto v1 = sph->getParticleReferences().back().getPosition();
	EXPECT_DOUBLE_EQ(x0[1], x1[1]);
	EXPECT_DOUBLE_EQ(v0[1], v1[1]);
	double realDistance = (x0 - x1).norm();
	EXPECT_DOUBLE_EQ(distance, realDistance);
}

TEST(SphRepresentationTest, DoUpdate2ParticlesAttractingTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = h * 3.0 / 4.0;
	auto sph = set2ParticlesInteracting(h, distance);

	auto x0 = sph->getParticleReferences().front().getPosition();
	auto x1 = sph->getParticleReferences().back().getPosition();
	auto v0 = sph->getParticleReferences().front().getPosition();
	auto v1 = sph->getParticleReferences().back().getPosition();
	EXPECT_DOUBLE_EQ(x0[1], x1[1]);
	EXPECT_DOUBLE_EQ(v0[1], v1[1]);
	double finalDistance = (x0 - x1).norm();
	EXPECT_LT(finalDistance, distance);
}

TEST(SphRepresentationTest, DoUpdate2ParticlesRetractingTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = h * 1.0 / 4.0;
	auto sph = set2ParticlesInteracting(h, distance);

	auto x0 = sph->getParticleReferences().front().getPosition();
	auto x1 = sph->getParticleReferences().back().getPosition();
	auto v0 = sph->getParticleReferences().front().getPosition();
	auto v1 = sph->getParticleReferences().back().getPosition();
	EXPECT_DOUBLE_EQ(x0[1], x1[1]);
	EXPECT_DOUBLE_EQ(v0[1], v1[1]);
	double finalDistance = (x0 - x1).norm();
	EXPECT_GT(finalDistance, distance);
}

TEST(SphRepresentationTest, DoUpdate2ParticlesInEquilibriumTest)
{
	// If you note R the radius of 1 particle, then a volume of 1l covered by 50 particles requires:
	// (50 * 4/3.PI.R^3) = 0.001 => R = 0.01683890300960629672761734255721m
	// Let's take a smooth kernel support of 2R, anything above that distance is not interacting
	double h = 2.0 * 0.01683890300960629672761734255721;
	double distance = h / 2.0;
	auto sph = set2ParticlesInteracting(h, distance);

	auto x0 = sph->getParticleReferences().front().getPosition();
	auto x1 = sph->getParticleReferences().back().getPosition();
	auto v0 = sph->getParticleReferences().front().getPosition();
	auto v1 = sph->getParticleReferences().back().getPosition();
	EXPECT_DOUBLE_EQ(x0[1], x1[1]);
	EXPECT_DOUBLE_EQ(v0[1], v1[1]);
	double finalDistance = (x0 - x1).norm();
	EXPECT_NEAR(finalDistance, distance, pow(h, 2));
}

TEST(SphRepresentationTest, SerializationTest)
{
	typedef SurgSim::Particles::SphRepresentation::PlaneConstraint PlaneConstraint;

	auto sph = std::make_shared<SphRepresentation>("TestSphRepresentation");
	sph->setDensity(1.1);
	sph->setGasStiffness(2.2);
	sph->setGravity(SurgSim::Math::Vector3d::Ones());
	sph->setKernelSupport(3.3);
	sph->setMassPerParticle(4.4);
	sph->setMaxParticles(5);
	SphRepresentation::PlaneConstraint p;
	p.stiffness = 6.6;
	p.damping = 7.7;
	p.planeEquation.setLinSpaced(8.8, 9.9);
	sph->addPlaneConstraint(p);
	sph->setSurfaceTension(10.1);
	sph->setViscosity(11.11);

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
	auto planeConstraints = newRepresentation->getValue<std::vector<PlaneConstraint>>("PlaneConstraints");
	EXPECT_EQ(sph->getPlaneConstraints().size(), planeConstraints.size());
	EXPECT_EQ(1u, planeConstraints.size());
	EXPECT_TRUE(sph->getPlaneConstraints()[0].planeEquation.isApprox(planeConstraints[0].planeEquation));
	EXPECT_DOUBLE_EQ(sph->getPlaneConstraints()[0].stiffness, planeConstraints[0].stiffness);
	EXPECT_DOUBLE_EQ(sph->getPlaneConstraints()[0].damping, planeConstraints[0].damping);
	EXPECT_DOUBLE_EQ(sph->getSurfaceTension(),
		newRepresentation->getValue<double>("SurfaceTension"));
	EXPECT_DOUBLE_EQ(sph->getViscosity(), newRepresentation->getValue<double>("Viscosity"));
}

}; // namespace Particles
}; // namespace SurgSim
