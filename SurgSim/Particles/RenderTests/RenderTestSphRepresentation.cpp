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
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Emitter.h"
#include "SurgSim/Particles/ParticlesCollisionRepresentation.h"
#include "SurgSim/Particles/RandomSpherePointGenerator.h"
#include "SurgSim/Particles/RenderTests/RenderTest.h"
#include "SurgSim/Particles/SphRepresentation.h"


namespace SurgSim
{
namespace Particles
{

std::shared_ptr<Framework::SceneElement> createCube()
{
	auto element = std::make_shared<Framework::BasicSceneElement>("cube");

	auto mesh = std::make_shared<Math::MeshShape>();
	mesh->load("Cube.ply");

	auto collision = std::make_shared<Collision::ShapeCollisionRepresentation>("collision");
	collision->setShape(mesh);
	element->addComponent(collision);

	auto graphics = std::make_shared<Graphics::OsgMeshRepresentation>("graphics");
	graphics->setShape(mesh);
	graphics->setDrawAsWireFrame(true);
	element->addComponent(graphics);

	return element;
}

std::shared_ptr<Framework::SceneElement> createParticleSystem()
{
	auto element = std::make_shared<Framework::BasicSceneElement>("particles");

	// c.f. "Lagrangian Fluid Dynamics Using Smoothed Particle Hydrodynamics", Micky Kelager, January 9th 2006.
	// for input data to simulate water.
	auto particles = std::make_shared<SphRepresentation>("physics");
	particles->setMaxParticles(2000);
	particles->setMassPerParticle(0.02);
	particles->setDensity(998.29);
	particles->setGasStiffness(3.0);
	particles->setKernelSupport(0.0457);
	particles->setSurfaceTension(0.0728);
	particles->setViscosity(3.5);
	particles->setStiffness(1000.0);
	particles->setDamping(4.0);
	particles->setFriction(0.1);
	element->addComponent(particles);

	auto collision = std::make_shared<ParticlesCollisionRepresentation>("collision");
	particles->setCollisionRepresentation(collision);
	element->addComponent(collision);

	auto emitter = std::make_shared<Emitter>("emitter");
	emitter->setTarget(particles);
	emitter->setShape(std::make_shared<Math::SphereShape>(0.1));
	emitter->setMode(EMIT_MODE_SURFACE);
	emitter->setRate(2000.0);
	emitter->setLifetimeRange(std::make_pair(30000, 600000));
	emitter->setVelocityRange(std::make_pair(Math::Vector3d::Zero(), Math::Vector3d::Zero()));
	element->addComponent(emitter);

	auto graphics = std::make_shared<Graphics::OsgPointCloudRepresentation>("graphics");
	graphics->setColor(Math::Vector4d::Ones());
	graphics->setPointSize(3.0f);
	element->addComponent(graphics);

	auto graphicsUpdater = std::make_shared<Blocks::TransferParticlesToPointCloudBehavior>("particles to graphics");
	graphicsUpdater->setSource(particles);
	graphicsUpdater->setTarget(graphics);
	element->addComponent(graphicsUpdater);

	return element;
}

TEST_F(RenderTests, SphRenderTest)
{
	auto particles = createParticleSystem();
	particles->setPose(Math::makeRigidTranslation(Math::Vector3d(0.0, 0.1, 0.0)));
	scene->addSceneElement(particles);

	auto cube = createCube();
	cube->setPose(Math::makeRigidTranslation(Math::Vector3d(0.0, -1.5, 0.0)));
	scene->addSceneElement(cube);

	physicsManager->setRate(500.0);

	runTest(Math::Vector3d(0.0, 0.0, 8.5), Math::Vector3d::Zero(), 20000.0);
}

};
};
