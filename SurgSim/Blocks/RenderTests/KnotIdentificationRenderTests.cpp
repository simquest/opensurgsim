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

///\file KnotIdentificationRenderTests.cpp render test for knot identification behavior

#include <gtest/gtest.h>

#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

using SurgSim::Blocks::KnotIdentificationBehavior;
using SurgSim::Math::Vector3d;

namespace
{
std::shared_ptr<SurgSim::Framework::SceneElement> makeSuture(const std::string& filename,
	SurgSim::Math::RigidTransform3d transform = SurgSim::Math::RigidTransform3d::Identity())
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Suture");

	// Physics
	auto physics = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
	physics->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
	physics->setLocalPose(SurgSim::Math::RigidTransform3d::Identity());
	physics->setLocalPose(transform);
	physics->loadFem(filename);
	physics->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
	physics->setLinearSolver(SurgSim::Math::LINEARSOLVER_LU);
	physics->setRayleighDampingMass(5.0);
	physics->setRayleighDampingStiffness(0.001);
	physics->setIsGravityEnabled(false);
	element->addComponent(physics);

	// Graphics
	auto graphics = std::make_shared<SurgSim::Graphics::OsgCurveRepresentation>("Graphics");
	graphics->setColor(SurgSim::Math::Vector4d(0.0, 0.0, 1.0, 1.0));
	graphics->setAntiAliasing(true);
	graphics->setWidth(0.9);
	graphics->setTension(0.0);
	element->addComponent(graphics);

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
	copier->setTarget(graphics);
	element->addComponent(copier);

	return element;
}

class KnotNameTextBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit KnotNameTextBehavior(const std::string& name)
		: SurgSim::Framework::Behavior(name)
	{

	}

	void setKnotIdBehavior(std::shared_ptr<SurgSim::Blocks::KnotIdentificationBehavior> knotId)
	{
		m_knotId = knotId;
	}

	bool doInitialize() override
	{
		if (m_knotId == nullptr)
		{
			return false;
		}
		return true;
	}

	bool doWakeUp() override
	{
		m_text = std::make_shared<SurgSim::Graphics::OsgTextRepresentation>("KnotName");
		m_text->setLocation(100.0, 100.0);
		m_text->setText("Waiting...");
		m_text->setColor(SurgSim::Math::Vector4d(1.0, 0.5, 0.5, 1.0));
		m_text->setDrawBackground(true);
		m_text->setBackgroundColor(SurgSim::Math::Vector4d(0.3, 0.3, 0.3, 1.0));
		getSceneElement()->addComponent(m_text);
		return true;
	}

	void update(double dt) override
	{
		m_text->setText(m_knotId->getKnotName());
	}

private:
	/// The osgText
	std::shared_ptr<SurgSim::Graphics::OsgTextRepresentation> m_text;
	/// The knot ID behavior
	std::shared_ptr<SurgSim::Blocks::KnotIdentificationBehavior> m_knotId;
};
}

class KnotIdentificationRenderTests : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

		graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
		runtime->addManager(graphicsManager);

		physicsManager = std::make_shared<SurgSim::Physics::PhysicsManager>();
		runtime->addManager(physicsManager);

		behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
		runtime->addManager(behaviorManager);

		scene = runtime->getScene();

		viewElement = std::make_shared<SurgSim::Graphics::OsgViewElement>("Physics Render Scene");
		scene->addSceneElement(viewElement);
	}

	virtual void TearDown()
	{
		runtime->stop();
	}

	virtual void runTest(const SurgSim::Math::Vector3d& cameraPosition, const SurgSim::Math::Vector3d& cameraLookAt,
		double miliseconds)
	{
		using SurgSim::Graphics::OsgAxesRepresentation;

		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(cameraPosition, cameraLookAt);

		auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Other");
		std::shared_ptr<OsgAxesRepresentation> axes = std::make_shared<OsgAxesRepresentation>("axes");
		axes->setSize(1.0);
		element->addComponent(axes);
		scene->addSceneElement(element);

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(static_cast<int>(miliseconds)));
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager;
	std::shared_ptr<SurgSim::Physics::PhysicsManager> physicsManager;
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<SurgSim::Graphics::OsgViewElement> viewElement;
};

TEST_F(KnotIdentificationRenderTests, TrefoilKnot)
{
	auto suture = makeSuture("Geometry/trefoil_knot.ply");
	auto knotId = std::make_shared<SurgSim::Blocks::KnotIdentificationBehavior>("KnotId");
	knotId->setFem1d(suture->getComponent("Physics"));
	suture->addComponent(knotId);
	auto knotText = std::make_shared<KnotNameTextBehavior>("KnotNameTextBehavior");
	knotText->setKnotIdBehavior(knotId);
	suture->addComponent(knotText);

	scene->addSceneElement(suture);

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 2000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

TEST_F(KnotIdentificationRenderTests, SquareKnot)
{
	auto suture = makeSuture("Geometry/square_knot.ply");
	auto knotId = std::make_shared<SurgSim::Blocks::KnotIdentificationBehavior>("KnotId");
	knotId->setFem1d(suture->getComponent("Physics"));
	suture->addComponent(knotId);
	auto knotText = std::make_shared<KnotNameTextBehavior>("KnotNameTextBehavior");
	knotText->setKnotIdBehavior(knotId);
	suture->addComponent(knotText);

	scene->addSceneElement(suture);

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 2000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

TEST_F(KnotIdentificationRenderTests, GrannyKnot)
{
	auto suture = makeSuture("Geometry/granny_knot.ply");
	auto knotId = std::make_shared<SurgSim::Blocks::KnotIdentificationBehavior>("KnotId");
	knotId->setFem1d(suture->getComponent("Physics"));
	suture->addComponent(knotId);
	auto knotText = std::make_shared<KnotNameTextBehavior>("KnotNameTextBehavior");
	knotText->setKnotIdBehavior(knotId);
	suture->addComponent(knotText);

	scene->addSceneElement(suture);

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 2000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

TEST_F(KnotIdentificationRenderTests, SquareKnotRotated)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationMatrix;
	using SurgSim::Math::Vector3d;
	Vector3d axis;
	axis << 0.42621342245242971, 0.69680137496288352, 0.57689683858660712;
	double angle = 4.3501990572973357;
	auto suture = makeSuture("Geometry/square_knot.ply",
		makeRigidTransform(makeRotationMatrix(angle, axis), Vector3d::Zero()));
	auto knotId = std::make_shared<SurgSim::Blocks::KnotIdentificationBehavior>("KnotId");
	knotId->setFem1d(suture->getComponent("Physics"));
	suture->addComponent(knotId);
	auto knotText = std::make_shared<KnotNameTextBehavior>("KnotNameTextBehavior");
	knotText->setKnotIdBehavior(knotId);
	suture->addComponent(knotText);

	scene->addSceneElement(suture);

	SurgSim::Math::Vector3d cameraPosition(0.0, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 2000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}

TEST_F(KnotIdentificationRenderTests, GrannyKnotRotated)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRotationMatrix;
	using SurgSim::Math::Vector3d;
	Vector3d axis;
	axis << 0.69882389249458876, 0.14364554633107285, 0.7007218594406478;
	double angle = 3.0412064700110828;
	auto suture = makeSuture("Geometry/granny_knot.ply",
		makeRigidTransform(makeRotationMatrix(angle, axis), Vector3d::Zero()));
	auto knotId = std::make_shared<SurgSim::Blocks::KnotIdentificationBehavior>("KnotId");
	knotId->setFem1d(suture->getComponent("Physics"));
	suture->addComponent(knotId);
	auto knotText = std::make_shared<KnotNameTextBehavior>("KnotNameTextBehavior");
	knotText->setKnotIdBehavior(knotId);
	suture->addComponent(knotText);

	scene->addSceneElement(suture);

	SurgSim::Math::Vector3d cameraPosition(0.0, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 2000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}