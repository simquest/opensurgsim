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

#include <memory>
#include <array>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgPlaneRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/View.h"
#include "SurgSim/Graphics/ViewElement.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;

namespace
{
std::shared_ptr<SurgSim::Framework::SceneElement> createSphereObject()
{

	auto result = std::make_shared<SurgSim::Framework::BasicSceneElement>("Object");

	auto physics = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
	physics->setDensity(700.0); // Wood
	physics->setLinearDamping(0.1);

	auto sphere = std::make_shared<SurgSim::Math::SphereShape>(0.01); // 1cm Sphere
	auto shape = std::make_shared<SurgSim::Math::CompoundShape>();
	physics->setShape(shape);
	result->addComponent(physics);

	std::array<SurgSim::Math::Vector3d, 6> offsets =
	{
		Vector3d(0.1, 0.0, 0.0),
		Vector3d(-0.1, 0.0, 0.0),
		Vector3d(0.0, 0.1, 0.0),
		Vector3d(0.0, -0.1, 0.0),
		Vector3d(0.0, 0.0, 0.1),
		Vector3d(0.0, 0.0, -0.1)
	};

	int i = 0;
	for (auto offset : offsets)
	{
		auto transform = SurgSim::Math::makeRigidTranslation(offset);
		shape->addShape(sphere, transform);

		auto graphics = std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("Graphics-" + std::to_string(i));
		graphics->setRadius(sphere->getRadius());

		graphics->setLocalPose(transform);
		result->addComponent(graphics);

		++i;
	}

	auto rigidCollision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
	physics->setCollisionRepresentation(rigidCollision);
	result->addComponent(rigidCollision);

	auto transform = SurgSim::Math::makeRigidTransform(
						 SurgSim::Math::makeRotationQuaternion(0.01, Vector3d(1.0, 1.0, 1.0)),
						 Vector3d(0.0, 1.0, 0.0));
	result->setPose(transform);

	return result;
};

std::shared_ptr<SurgSim::Framework::SceneElement> createMeshObject(
	std::shared_ptr<SurgSim::Math::CompoundShape>* compoundShapeOut = nullptr,
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation>* leftGraphicsOut = nullptr,
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation>* rightGraphicsOut = nullptr)
{
	SurgSim::Framework::ApplicationData data("config.txt");

	auto result = std::make_shared<SurgSim::Framework::BasicSceneElement>("Object");

	auto physics = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
	physics->setDensity(700.0); // Wood
	physics->setLinearDamping(0.5);

	auto subShape = std::make_shared<SurgSim::Math::MeshShape>();
	subShape->load("bar.ply", data);
	auto shape = std::make_shared<SurgSim::Math::CompoundShape>();

	if (compoundShapeOut != nullptr)
	{
		*compoundShapeOut = shape;
	}

	physics->setShape(shape);
	result->addComponent(physics);

	auto transform = SurgSim::Math::makeRigidTransform(
						 SurgSim::Math::makeRotationQuaternion(0.2, Vector3d(1.0, 0.0, 0.0)),
						 Vector3d(0.0, 0.0, 0.0));
	shape->addShape(subShape, transform);

	auto graphics = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("LeftGraphics");
	graphics->setLocalPose(transform);
	graphics->loadModel("bar.ply");
	result->addComponent(graphics);

	if (leftGraphicsOut != nullptr)
	{
		*leftGraphicsOut = graphics;
	}

	transform = SurgSim::Math::makeRigidTransform(
					SurgSim::Math::makeRotationQuaternion(-0.2, Vector3d(1.0, 0.0, 0.0)),
					Vector3d(0.0, 0.0, 0.0));
	shape->addShape(subShape->getTransformed(SurgSim::Math::RigidTransform3d::Identity()), transform);

	graphics = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("RightGraphics");
	graphics->setLocalPose(transform);
	graphics->loadModel("bar.ply");
	result->addComponent(graphics);

	if (rightGraphicsOut != nullptr)
	{
		*rightGraphicsOut = graphics;
	}

	auto rigidCollision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
	physics->setCollisionRepresentation(rigidCollision);
	result->addComponent(rigidCollision);

	transform = SurgSim::Math::makeRigidTransform(
					SurgSim::Math::makeRotationQuaternion(0.3, Vector3d(1.0, 1.0, 1.0)),
					Vector3d(0.0, 1.0, 0.0));
	result->setPose(transform);

	return result;
};

std::vector <std::shared_ptr<SurgSim::Framework::SceneElement>> makeScenery()
{
	std::vector <std::shared_ptr<SurgSim::Framework::SceneElement>> result;

	{
		// Plane
		auto shape = std::make_shared<SurgSim::Math::PlaneShape>();
		auto physics = std::make_shared<SurgSim::Physics::FixedRepresentation>("Physics");
		physics->setShape(shape);

		auto graphics = std::make_shared<SurgSim::Graphics::OsgPlaneRepresentation>("Graphics");
		auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
		physics->setCollisionRepresentation(collision);

		auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Plane");
		element->addComponent(physics);
		element->addComponent(collision);
		element->addComponent(graphics);
		result.push_back(element);
	}

	{
		// Sphere
		auto shape = std::make_shared<SurgSim::Math::SphereShape>(2.0);
		auto physics = std::make_shared<SurgSim::Physics::FixedRepresentation>("Physics");
		physics->setShape(shape);

		auto graphics = std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("Graphics");
		graphics->setRadius(2.0);
		auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
		physics->setCollisionRepresentation(collision);

		auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("BigSphere");
		element->addComponent(physics);
		element->addComponent(collision);
		element->addComponent(graphics);

		auto transform = SurgSim::Math::makeRigidTransform(
							 SurgSim::Math::makeRotationQuaternion(0.01, Vector3d(1.0, 1.0, 1.0)),
							 Vector3d(0.0, -2.0, 0.0));
		element->setPose(transform);
		result.push_back(element);
	}

	{
		// Mesh Box
		auto shape = std::make_shared<SurgSim::Math::MeshShape>();
		shape->load("collider.ply");
		auto physics = std::make_shared<SurgSim::Physics::FixedRepresentation>("Physics");
		physics->setDensity(700.0); // Wood
		physics->setLinearDamping(0.1);
		physics->setShape(shape);

		auto graphics = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("Graphics");
		graphics->loadModel("collider.ply");
		auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
		physics->setCollisionRepresentation(collision);

		auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("MeshBox");
		element->addComponent(physics);
		element->addComponent(collision);
		element->addComponent(graphics);

		result.push_back(element);
	}


	return result;
}

class RotateSubshapeBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit RotateSubshapeBehavior(const std::string& name = "RotateSubshapeBehavior") :
		m_angle(0.2), m_velocity(0.05), SurgSim::Framework::Behavior(name)
	{
	}
	
	~RotateSubshapeBehavior()
	{
	}

	void setShape(std::shared_ptr<SurgSim::Math::CompoundShape> shape)
	{
		m_shape = shape;
	}

	void setGraphics(std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> leftGraphics,
		std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> rightGraphics)
	{
		m_leftGraphics = leftGraphics;
		m_rightGraphics = rightGraphics;
	}

	void update(double dt) override
	{
		//std::cout << "Starting to update poses" << std::endl;
		m_angle += m_velocity * dt;
		auto transform = SurgSim::Math::makeRigidTransform(
			SurgSim::Math::makeRotationQuaternion(m_angle, Vector3d(1.0, 0.0, 0.0)),
			Vector3d(0.0, 0.0, 0.0));
		m_shape->setPose(0, transform);
		m_leftGraphics->setLocalPose(transform);

		transform = SurgSim::Math::makeRigidTransform(
			SurgSim::Math::makeRotationQuaternion(-m_angle, Vector3d(1.0, 0.0, 0.0)),
			Vector3d(0.0, 0.0, 0.0));

		m_shape->setPose(1, transform);
		m_rightGraphics->setLocalPose(transform);
		//std::cout << "Done updating poses" << std::endl;
	}

protected:
	bool doInitialize() override
	{
		return true;
	}
	bool doWakeUp() override
	{
		return true;
	}

private:
	double m_angle;
	double m_velocity;
	std::shared_ptr<SurgSim::Math::CompoundShape> m_shape;
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> m_leftGraphics;
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> m_rightGraphics;
};

}

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, CompoundCollisionShapePlane)
{
	auto sphereObject = createSphereObject();
	scene->addSceneElement(sphereObject);
	sphereObject->getComponent("Collision")->wakeUp();
	auto scenery = makeScenery()[0];
	scene->addSceneElement(scenery);
	scenery->getComponent("Collision")->wakeUp();

	runTest(Vector3d(0.8, 0.8, 0.8), Vector3d(0.0, 0.0, 0.0), 5000);
}

TEST_F(RenderTests, CompoundCollisionShapeSphere)
{
	auto sphereObject = createSphereObject();
	scene->addSceneElement(sphereObject);
	sphereObject->getComponent("Collision")->wakeUp();
	auto scenery = makeScenery()[1];
	scene->addSceneElement(scenery);
	scenery->getComponent("Collision")->wakeUp();

	runTest(Vector3d(0.8, 0.8, 0.8), Vector3d(0.0, 0.0, 0.0), 5000);
}

TEST_F(RenderTests, CompoundCollisionMeshPlane)
{
	auto meshObject = createMeshObject();
	scene->addSceneElement(meshObject);
	meshObject->getComponent("Collision")->wakeUp();
	auto scenery = makeScenery()[0];
	scene->addSceneElement(scenery);
	scenery->getComponent("Collision")->wakeUp();

	runTest(Vector3d(0.8, 0.8, 0.8), Vector3d(0.0, 0.0, 0.0), 5000);
}

TEST_F(RenderTests, CompoundCollisionMeshBox)
{
	auto meshObject = createMeshObject();
	scene->addSceneElement(meshObject);
	meshObject->getComponent("Collision")->wakeUp();
	auto scenery = makeScenery()[2];
	scene->addSceneElement(scenery);
	scenery->getComponent("Collision")->wakeUp();

	runTest(Vector3d(0.8, 0.8, 0.8), Vector3d(0.0, 0.0, 0.0), 5000);
}

TEST_F(RenderTests, CompoundCollisionMoving)
{
	std::shared_ptr<SurgSim::Math::CompoundShape> shape;
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> leftGraphics;
	std::shared_ptr<SurgSim::Graphics::SceneryRepresentation> rightGraphics;

	auto element = createMeshObject(&shape, &leftGraphics, &rightGraphics);

	auto transform = SurgSim::Math::makeRigidTransform(
						 SurgSim::Math::makeRotationQuaternion(0.0, Vector3d(1.0, 0.0, 0.0)),
						 Vector3d(0.0, 0.5, 0.0));
	element->setPose(transform);
	scene->addSceneElement(element);
	element->getComponent("Collision")->wakeUp();

	auto scenery = makeScenery()[0];
	scene->addSceneElement(scenery);
	scenery->getComponent("Collision")->wakeUp();

	viewElement->enableManipulator(true);
	viewElement->setManipulatorParameters(Vector3d(0.8, 0.8, 0.8), Vector3d(0.0, 0.0, 0.0));

	runtime->start();

	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
	auto behavior = std::make_shared<RotateSubshapeBehavior>();
	behavior->setShape(shape);
	behavior->setGraphics(leftGraphics, rightGraphics);
	auto behaviorElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("BehaviorElement");
	behaviorElement->addComponent(behavior);
	scene->addSceneElement(behaviorElement);
	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
}

}
}
