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

///\file RenderTestSphRepresentation.cpp render test for SphRepresentation

#include <memory>

#include "SurgSim/Blocks/TransferParticlesToPointCloudBehavior.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/EmitterRepresentation.h"
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"
#include "SurgSim/Particles/RandomSpherePointGenerator.h"
#include "SurgSim/Particles/RenderTests/RenderTest.h"
#include "SurgSim/Particles/SphRepresentation.h"

using SurgSim::Blocks::TransferParticlesToPointCloudBehavior;
using SurgSim::Framework::Behavior;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Particles::RenderTests;
using SurgSim::Particles::SphRepresentation;

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createCube()
{
	auto mesh = std::make_shared<SurgSim::Math::MeshShape>();
	mesh->load("Cube.ply");

	auto collision = std::make_shared<SurgSim::Collision::ShapeCollisionRepresentation>("collision");
	collision->setShape(mesh);

	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("graphics");
	graphics->setShape(mesh);
	graphics->setDrawAsWireFrame(true);

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("cube");
	element->addComponent(collision);
	element->addComponent(graphics);

	return element;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createParticleSystem()
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("particles");

	// c.f. "Lagrangian Fluid Dynamics Using Smoothed Particle Hydrodynamics", Micky Kelager, January 9th 2006.
	// for input data to simulate water.
	auto particles = std::make_shared<SurgSim::Particles::SphRepresentation>("physics");
	particles->setMaxParticles(5000);
	particles->setMassPerParticle(0.02);
	particles->setDensity(998.29);
	particles->setGasStiffness(3.0);
	particles->setGravity(SurgSim::Math::Vector3d(0.0, -9.81, 0.0));
	particles->setKernelSupport(0.0457);
	particles->setSurfaceTension(0.0728);
	particles->setViscosity(3.5);
	element->addComponent(particles);

	auto collision = std::make_shared<SurgSim::Particles::ParticlesCollisionRepresentation>("collision");
	particles->setCollisionRepresentation(collision);
	element->addComponent(collision);

	//SphRepresentation::PlaneConstraint planeConstraint;
	//planeConstraint.stiffness = 50000.0;
	//planeConstraint.damping = 100.0;
	//planeConstraint.planeEquation = SurgSim::Math::Vector4d(0.0, 1.0, 0.0, 0.0);
	//particles->addPlaneConstraint(planeConstraint);
	//planeConstraint.planeEquation.segment<3>(0) = Vector3d(-1,0.9,0).normalized();
	//particles->addPlaneConstraint(planeConstraint);
	//planeConstraint.planeEquation.segment<3>(0) = Vector3d(+1,0.9,0).normalized();
	//particles->addPlaneConstraint(planeConstraint);
	//planeConstraint.planeEquation.segment<3>(0) = Vector3d(0,0.9,+1).normalized();
	//particles->addPlaneConstraint(planeConstraint);
	//planeConstraint.planeEquation.segment<3>(0) = Vector3d(0,0.9,-1).normalized();
	//particles->addPlaneConstraint(planeConstraint);

	auto sphereEmitter = std::make_shared<SurgSim::Particles::EmitterRepresentation>("emitter");
	sphereEmitter->setLocalPose(SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.5, 1.0, 0.5)));
	sphereEmitter->setTarget(particles);
	sphereEmitter->setShape(std::make_shared<SurgSim::Math::SphereShape>(0.1));
	sphereEmitter->setMode(SurgSim::Particles::EMIT_MODE_SURFACE);
	sphereEmitter->setRate(200.0); /// 2000 particles per second (maximum is 5000)
	sphereEmitter->setLifetimeRange(std::make_pair(0.2, 0.4));
	sphereEmitter->setVelocityRange(std::make_pair(SurgSim::Math::Vector3d::Zero(), SurgSim::Math::Vector3d::Zero()));
	element->addComponent(sphereEmitter);

	auto graphics = std::make_shared<OsgPointCloudRepresentation>("graphics");
	graphics->setColor(Vector4d::Ones());
	graphics->setPointSize(3.0f);
	element->addComponent(graphics);

	auto particlesToGraphics =
		std::make_shared<SurgSim::Blocks::TransferParticlesToPointCloudBehavior>("particles to graphics");
	particlesToGraphics->setSource(particles);
	particlesToGraphics->setTarget(graphics);
	element->addComponent(particlesToGraphics);

	return element;
}

};

TEST_F(RenderTests, SphRenderTest)
{
	auto particles = createParticleSystem();
	scene->addSceneElement(particles);

	auto cube = createCube();
	cube->setPose(SurgSim::Math::makeRigidTranslation(Vector3d(0.0, -0.2, 0.0)));
	scene->addSceneElement(cube);

	// Particle manager runs at 500Hz
	particlesManager->setRate(500.0);

	runTest(Vector3d(0.0, 0.0, 8.5), Vector3d::Zero(), 200000.0);
}
