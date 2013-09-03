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

#include <memory>
#include <boost/thread.hpp>

#include <SurgSim/Blocks/BasicSceneElement.h>
#include <SurgSim/Blocks/RepresentationPoseBehavior.h>
#include <SurgSim/Devices/MultiAxis/MultiAxisDevice.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgPlaneRepresentation.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Input/InputManager.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>
#include <SurgSim/Physics/SphereShape.h>
#include <SurgSim/Physics/RigidCollisionRepresentation.h>
#include <SurgSim/Physics/RigidShapeCollisionRepresentation.h>
#include <SurgSim/Physics/VtcRigidParameters.h>
#include <SurgSim/Physics/VtcRigidRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::RepresentationPoseBehavior;
using SurgSim::Blocks::RepresentationPoseBehavior2;
using SurgSim::Blocks::InputVtcBehavior;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::SphereShape;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::VtcRigidParameters;
using SurgSim::Physics::VtcRigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;


std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	return viewElement;
}

std::shared_ptr<SceneElement> createPlane(const std::string& name,
										  const SurgSim::Math::RigidTransform3d& pose)
{
	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(name + " Physics");

	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<OsgPlaneRepresentation> graphicsRepresentation =
		std::make_shared<OsgPlaneRepresentation>(name + " Graphics");
	graphicsRepresentation->setInitialPose(pose);

	std::shared_ptr<SurgSim::Physics::DoubleSidedPlaneShape> planeShape =
		std::make_shared<SurgSim::Physics::DoubleSidedPlaneShape>();

	std::shared_ptr<SceneElement> planeElement = std::make_shared<BasicSceneElement>(name);
	planeElement->addComponent(physicsRepresentation);
	planeElement->addComponent(graphicsRepresentation);

	planeElement->addComponent(std::make_shared<RepresentationPoseBehavior>("Physics to Graphics Pose",
								physicsRepresentation, graphicsRepresentation));
	planeElement->addComponent(std::make_shared<SurgSim::Physics::RigidShapeCollisionRepresentation>
								("Plane Collision",planeShape, physicsRepresentation));
	return planeElement;
}


std::shared_ptr<SceneElement> createSphere(const std::string& name)
{
	RigidRepresentationParameters params;
	params.setDensity(700.0); // Wood
	params.setLinearDamping(0.1);

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.5); // 5cm Sphere
	params.setShapeUsedForMassInertia(shape);

	VtcRigidParameters vtcParams;
	vtcParams.setVtcAngularDamping(1);
	vtcParams.setVtcAngularStiffness(1);
	vtcParams.setVtcLinearDamping(1);
	vtcParams.setVtcLinearStiffness(1);

	std::shared_ptr<VtcRigidRepresentation> vtcRepresentation =
		std::make_shared<VtcRigidRepresentation>(name + "-Vtc");
	vtcRepresentation->setInitialParameters(params);
	vtcRepresentation->setInitialVtcParameters(vtcParams);

	std::shared_ptr<OsgSphereRepresentation> graphicsRepresentation =
		std::make_shared<OsgSphereRepresentation>(name + "-Graphics");
	graphicsRepresentation->setRadius(shape->getRadius());

	std::shared_ptr<OsgSphereRepresentation> graphicsRepresentation2 =
		std::make_shared<OsgSphereRepresentation>(name + "2-Graphics");
	graphicsRepresentation2->setRadius(shape->getRadius());

	std::shared_ptr<SurgSim::Input::InputComponent> inputComponent =
		std::make_shared<SurgSim::Input::InputComponent>("input", "MultiAxisDevice");

	std::shared_ptr<SceneElement> sphereElement = std::make_shared<BasicSceneElement>(name);
	sphereElement->addComponent(vtcRepresentation);
	sphereElement->addComponent(graphicsRepresentation);
	sphereElement->addComponent(graphicsRepresentation2);
	sphereElement->addComponent(inputComponent);

	sphereElement->addComponent(std::make_shared<InputVtcBehavior>("Input to Vtc",
								inputComponent, vtcRepresentation));
	sphereElement->addComponent(std::make_shared<RepresentationPoseBehavior>("Physics to Graphics Pose",
								vtcRepresentation, graphicsRepresentation));
	sphereElement->addComponent(std::make_shared<RepresentationPoseBehavior2>("Physics to Graphics2 Pose",
								vtcRepresentation, graphicsRepresentation2));
	//sphereElement->addComponent(std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>
								//("Sphere Collision Representation", vtcRepresentation));
	return sphereElement;
}


int main(int argc, char* argv[])
{
	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	std::shared_ptr<SurgSim::Input::InputManager> inputManager = std::make_shared<SurgSim::Input::InputManager>();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::shared_ptr<SurgSim::Device::MultiAxisDevice> toolDevice =
		std::make_shared<SurgSim::Device::MultiAxisDevice>("MultiAxisDevice");
	toolDevice->setPositionScale(0.0003);
	toolDevice->setOrientationScale(0.0003);
	SURGSIM_ASSERT( toolDevice->initialize() == true ) <<
		"Could not initialize device '%s' for the tool.\n", toolDevice->getName().c_str();

	inputManager->addDevice(toolDevice);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());
	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());
	scene->addSceneElement(createSphere("sphere1"));
	scene->addSceneElement(createPlane("plane1",
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.0))));
	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	runtime->setScene(scene);
	runtime->execute();

	return 0;
}
