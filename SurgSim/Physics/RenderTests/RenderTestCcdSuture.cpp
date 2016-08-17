// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

///\file RenderTestCcdSuture.cpp render test for ccd with Fem1D

#include <memory>

#include "SurgSim/Blocks/CompoundShapeToGraphics.h"
#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"
#include "SurgSim/Blocks/VisualizeConstraints.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"


namespace
{
std::shared_ptr<SurgSim::Framework::SceneElement> makeSuture(const std::string& filename)
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Suture");

	// Physics
	auto physics = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
	physics->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
	physics->setLocalPose(SurgSim::Math::RigidTransform3d::Identity());
	physics->loadFem(filename);
	physics->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
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
	shape->setRadius(0.001);
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

std::shared_ptr<SurgSim::Framework::SceneElement> makeRigid(const std::string& filename)
{
	SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.0, -0.1, 0.0));

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("RigidMesh");
	element->setPose(pose);
	auto shape = std::make_shared<SurgSim::Math::MeshShape>();
	shape->load(filename);

	auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
	rigid->setIsGravityEnabled(false);
	// http://www.engineeringtoolbox.com/wood-density-d_40.html
	rigid->setDensity(5800.0); // Cedar of Lebanon wood density 5800.0 Kg/m-3
	rigid->setShape(shape);
	element->addComponent(rigid);

	auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
	collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	rigid->setCollisionRepresentation(collision);
	collision->setShape(shape);
	element->addComponent(collision);

	auto osgRepresentation = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("Graphics");
	osgRepresentation->setShape(shape);
	element->addComponent(osgRepresentation);

	return element;
}

std::shared_ptr<SurgSim::Framework::SceneElement> makeCompound()
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Compound");
	using SurgSim::Math::Vector3d;

	auto physics = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
	physics->setDensity(700.0); // Wood
	physics->setLinearDamping(0.5);
	physics->setIsGravityEnabled(false);

	auto subShape = std::make_shared<SurgSim::Math::MeshShape>();
	subShape->load("bar.ply");
	auto shape = std::make_shared<SurgSim::Math::CompoundShape>();

	physics->setShape(shape);
	element->addComponent(physics);

	auto transform = SurgSim::Math::makeRigidTransform(
						 SurgSim::Math::makeRotationQuaternion(0.2, Vector3d(1.0, 0.0, 0.0)),
						 Vector3d(0.0, 0.0, 0.0));
	shape->addShape(subShape, transform);

	auto copier = std::make_shared<SurgSim::Blocks::CompoundShapeToGraphics>("Copier");
	copier->setSource(physics);
	element->addComponent(copier);

	auto graphics = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("LeftGraphics");
	graphics->setLocalPose(transform);
	graphics->loadModel("bar.ply");
	element->addComponent(graphics);
	copier->addTarget(graphics);


	transform = SurgSim::Math::makeRigidTransform(
					SurgSim::Math::makeRotationQuaternion(-0.2, Vector3d(1.0, 0.0, 0.0)),
					Vector3d(0.0, 0.0, 0.0));
	shape->addShape(subShape, transform);

	graphics = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("RightGraphics");
	graphics->setLocalPose(transform);
	graphics->loadModel("bar.ply");
	element->addComponent(graphics);
	copier->addTarget(graphics);

	auto rigidCollision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
	physics->setCollisionRepresentation(rigidCollision);
	rigidCollision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);

	element->addComponent(rigidCollision);

	transform = SurgSim::Math::makeRigidTransform(
					SurgSim::Math::makeRotationQuaternion(M_PI_2, Vector3d(0.0, 1.0, 0.0)),
					Vector3d(0.0, -0.2, 0.0));
	element->setPose(transform);

	return element;
}

}

class CcdSutureTest : public SurgSim::Physics::RenderTests
{
public:
	void SetUp() override
	{
		SurgSim::Physics::RenderTests::SetUp();
		scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

		SurgSim::Framework::Logger::getLoggerManager()->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
		physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());
		physicsManager->setRate(150.0);
	}
};

TEST_F(CcdSutureTest, SutureVsMeshedCylinder)
{
	scene->addSceneElement(makeSuture("prolene 3.0-fixedExtremity.ply"));
	scene->addSceneElement(makeRigid("cylinder.ply"));

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.1);
	SurgSim::Math::Vector3d cameraLookAt(0.0, -0.1, 0.0);
	physicsManager->setRate(100.0);
	double milliseconds = 5000.0;
	runTest(cameraPosition, cameraLookAt, milliseconds);
}

TEST_F(CcdSutureTest, Fem1DHalfKnot)
{
	scene->addSceneElement(makeSuture("half_knot.ply"));

	SurgSim::Math::Vector3d cameraPosition(1.0, 0.0, 1.0);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	physicsManager->setRate(150.0);
	double milliseconds = 5000.0;
	runTest(cameraPosition, cameraLookAt, milliseconds);
}

TEST_F(CcdSutureTest, Fem1DLoop)
{
	scene->addSceneElement(makeSuture("loop.ply"));

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	physicsManager->setRate(50.0);
	double milliseconds = 5000.0;
	runTest(cameraPosition, cameraLookAt, milliseconds);
}

TEST_F(CcdSutureTest, Fem1DBlock)
{

	std::shared_ptr<SurgSim::Framework::SceneElement> element =
		std::make_shared<SurgSim::Framework::BasicSceneElement>("Axis");
	element->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("Axis"));
	scene->addSceneElement(element);

	element = makeRigid("bar.ply");
	auto transform = SurgSim::Math::makeRigidTransform(
						 SurgSim::Math::makeRotationQuaternion(M_PI_2, SurgSim::Math::Vector3d(0.0, 1.0, 0.0)),
						 SurgSim::Math::Vector3d(0.0, -0.2, 0.0));
	element->setPose(transform);
	scene->addSceneElement(element);
	scene->addSceneElement(makeSuture("prolene 3.0-fixedExtremity.ply"));

	physicsManager->setRate(100.0);
	physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());
	scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, -0.1, 0.0);
	double miliseconds = 5000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}


TEST_F(CcdSutureTest, Fem1DCompound)
{

	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Axis");
	element->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("Axis"));
	scene->addSceneElement(element);

	scene->addSceneElement(makeCompound());

	scene->addSceneElement(makeSuture("prolene 3.0-fixedExtremity.ply"));

	physicsManager->setRate(100.0);
	physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());
	scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, -0.1, 0.0);
	double miliseconds = 5000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}


TEST_F(CcdSutureTest, Fem1DMovingCompound)
{

	using SurgSim::Math::Vector3d;
	std::shared_ptr<SurgSim::Framework::SceneElement> element =
		std::make_shared<SurgSim::Framework::BasicSceneElement>("Axis");
	element->addComponent(std::make_shared<SurgSim::Graphics::OsgAxesRepresentation>("Axis"));
	scene->addSceneElement(element);

	element = makeCompound();

	auto leftGraphics =
		std::dynamic_pointer_cast<SurgSim::Graphics::Representation>(element->getComponent("LeftGraphics"));
	auto rightGraphics =
		std::dynamic_pointer_cast<SurgSim::Graphics::Representation>(element->getComponent("RightGraphics"));

	auto physics = std::dynamic_pointer_cast<SurgSim::Physics::RigidRepresentation>(element->getComponent("Physics"));

	ASSERT_NE(nullptr, leftGraphics);
	ASSERT_NE(nullptr, rightGraphics);
	ASSERT_NE(nullptr, physics);


	auto shape = std::dynamic_pointer_cast<SurgSim::Math::CompoundShape>(physics->getShape());

	scene->addSceneElement(element);

	scene->addSceneElement(makeSuture("prolene 3.0-fixedExtremity.ply"));

	physicsManager->setRate(300.0);
	physicsManager->setComputations(SurgSim::Physics::createCcdPipeline());
	scene->addSceneElement(std::make_shared<SurgSim::Blocks::VisualizeConstraints>());

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, -0.1, 0.0);


	runtime->start();
	viewElement->enableManipulator(true);
	viewElement->setManipulatorParameters(cameraPosition, cameraLookAt);

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	for (double angle = 0.2, offset = -0.05; angle < 1.0; angle += 0.00125, offset += 0.0005)
	{
		auto transform = SurgSim::Math::makeRigidTransform(
							 SurgSim::Math::makeRotationQuaternion(angle, Vector3d(1.0, 0.0, 0.0)),
							 Vector3d(0.0, offset, 0.0));
		shape->setPose(0, transform);
		leftGraphics->setLocalPose(transform);

		transform = SurgSim::Math::makeRigidTransform(
						SurgSim::Math::makeRotationQuaternion(-angle, Vector3d(1.0, 0.0, 0.0)),
						Vector3d(0.0, offset, 0.0));

		shape->setPose(1, transform);
		rightGraphics->setLocalPose(transform);
		boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	}
}



