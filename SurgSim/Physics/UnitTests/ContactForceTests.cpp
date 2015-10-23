#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"


namespace SurgSim
{
namespace Physics
{

TEST(ContactForceTests, BoxPlane)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	runtime->addManager(std::make_shared<Physics::PhysicsManager>());
	auto scene = runtime->getScene();

	auto sphere = std::make_shared<Framework::BasicSceneElement>("Box");
	auto spherePhysics = std::make_shared<RigidRepresentation>("Physics");
	auto sphereCollider = std::make_shared<RigidCollisionRepresentation>("Collision");
	auto spherePose = Math::makeRigidTranslation(Math::Vector3d(0.0, 1.0, 0.0));
	spherePhysics->setShape(std::make_shared<Math::SphereShape>(1.0));
	spherePhysics->setDensity(10.0);
	spherePhysics->setCollisionRepresentation(sphereCollider);
	sphere->addComponent(sphereCollider);
	sphere->addComponent(spherePhysics);
	sphere->setPose(spherePose);
	scene->addSceneElement(sphere);

	auto plane = std::make_shared<Framework::BasicSceneElement>("Plane");
	auto planePhysics = std::make_shared<FixedRepresentation>("Physics");
	auto planeCollider = std::make_shared<RigidCollisionRepresentation>("Collision");
	planePhysics->setShape(std::make_shared<Math::PlaneShape>());
	planePhysics->setCollisionRepresentation(planeCollider);
	plane->addComponent(planeCollider);
	plane->addComponent(planePhysics);
	scene->addSceneElement(plane);

	runtime->start(true);
	for (int i = 0; i < 5000; i++)
	{
		runtime->step();
	}
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));
	EXPECT_TRUE(spherePose.isApprox(sphere->getPose(), 1e-4))
		<< "  Actual: " << std::endl << sphere->getPose().matrix() << std::endl
		<< "Expected: " << std::endl << spherePose.matrix();

	auto contactMap = planeCollider->getCollisions().safeGet();
	auto contacts = contactMap->find(sphereCollider);
	ASSERT_NE(contactMap->end(), contacts);
	
	Math::Vector3d expectedTotalForce = spherePhysics->getMass() * Math::Vector3d(0.0, -9.81, 0.0);
	Math::Vector3d totalForce = Math::Vector3d::Zero();
	for (auto contact : contacts->second)
	{
		totalForce += contact->force;
	}
	EXPECT_TRUE(expectedTotalForce.isApprox(totalForce))
		<< "  Actual: " << totalForce.transpose() << std::endl
		<< "Expected: " << expectedTotalForce.transpose();
	
	runtime->stop();
}


};
};
