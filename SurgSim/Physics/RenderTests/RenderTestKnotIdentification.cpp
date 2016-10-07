// This file is a part of the SimQuest OpenSurgSim extension.
// Copyright 2012-2016, SimQuest Solutions Inc.

///\file RenderTestKnotIdentification.cpp render test for knot identification behavior

#include <memory>

#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Blocks/KnotIdentificationBehavior.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Math::Vector3d;

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
}

class KnotNameTextBehavior : public SurgSim::Framework::Behavior
{
public:
	KnotNameTextBehavior(const std::string& name)
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
		m_text = std::make_shared<SurgSim::Graphics::OsgTextRepresentation>("KnotName");
		m_text->setLocation(100.0, 100.0);
		m_text->setText("Waiting...");
		m_text->setColor(SurgSim::Math::Vector4d(1.0, 0.5, 0.5, 1.0));
		m_text->setDrawBackground(true);
		m_text->setBackgroundColor(SurgSim::Math::Vector4d(0.3, 0.3, 0.3, 1.0));
		getSceneElement()->addComponent(m_text);
		return true;
	}

	bool doWakeUp() override
	{
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

class KnotIdentificationRenderTests : public SurgSim::Physics::RenderTests
{
public:
	void SetUp() override
	{
		SurgSim::Physics::RenderTests::SetUp();

		physicsManager->setRate(125.0);
		physicsManager->setComputations(SurgSim::Physics::createCcdPipeline(false));
	}
};

TEST_F(KnotIdentificationRenderTests, TrefoilKnot)
{
	auto suture = makeSuture("half_knot.ply");
	auto knotId = std::make_shared<SurgSim::Blocks::KnotIdentificationBehavior>("KnotId");
	knotId->setFem1d(suture->getComponent("Physics"));
	suture->addComponent(knotId);
	auto knotText = std::make_shared<KnotNameTextBehavior>("KnotNameTextBehavior");
	knotText->setKnotIdBehavior(knotId);
	suture->addComponent(knotText);

	scene->addSceneElement(suture);

	SurgSim::Math::Vector3d cameraPosition(0.25, 0.0, 0.25);
	SurgSim::Math::Vector3d cameraLookAt(0.0, 0.0, 0.0);
	double miliseconds = 100000.0;

	runTest(cameraPosition, cameraLookAt, miliseconds);
}
