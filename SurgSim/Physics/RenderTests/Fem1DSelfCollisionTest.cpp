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

#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

#include <gtest/gtest.h>

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> makeSuture(const std::string& filename)
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Fem1D");

	// Physics

	auto physics = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
	physics->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
	physics->setLocalPose(SurgSim::Math::RigidTransform3d::Identity());
	physics->loadFem(filename);
	physics->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT);
	physics->setLinearSolver(SurgSim::Math::LINEARSOLVER_LU);
	physics->setRayleighDampingMass(5.0);
	physics->setRayleighDampingStiffness(0.001);
	physics->setIsGravityEnabled(true);
	element->addComponent(physics);

	// Graphics
	auto gfx = std::make_shared<SurgSim::Graphics::OsgCurveRepresentation>("Graphics");
	gfx->setColor(SurgSim::Math::Vector4d(0.0, 0.0, 1.0, 1.0));
	gfx->setAntiAliasing(true);
	gfx->setWidth(0.9);
	element->addComponent(gfx);

	auto collision = std::make_shared<SurgSim::Physics::DeformableCollisionRepresentation>("Collision");
	auto shape = std::make_shared<SurgSim::Math::SegmentMeshShape>();
	shape->load(filename);
	shape->setRadius(0.0001);
	collision->setShape(shape);
	collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	collision->setSelfCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	physics->setCollisionRepresentation(collision);
	element->addComponent(collision);

	auto copier = std::make_shared<SurgSim::Blocks::TransferPhysicsToVerticesBehavior>("Copier");
	copier->setSource(physics);
	copier->setTarget(gfx);
	element->addComponent(copier);

	return element;
}
}

namespace SurgSim
{

namespace Physics
{

class Fem1DSelfCollisionTest : public SurgSim::Physics::RenderTests
{
};

TEST_F(Fem1DSelfCollisionTest, SimpleLoop)
{
	/// This uses a linear model, any large rotations won't exhibit normal behavior
	auto axes = std::make_shared<SurgSim::Framework::BasicSceneElement>("Axes");
	axes->addComponent(std::make_shared<Graphics::OsgAxesRepresentation>("Axes"));
	scene->addSceneElement(axes);

	scene->addSceneElement(makeSuture("loop.ply"));

	physicsManager->setRate(1000.0);

	SurgSim::Math::Vector3d cameraPosition(0.5, 0.0, 0.5);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 50000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

}
}