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

#include "SurgSim/Blocks/DriveElementFromInputBehavior.h"
#include "SurgSim/Devices/Devices.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

using SurgSim::Blocks::DriveElementFromInputBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgProgram;
using SurgSim::Graphics::OsgUniform;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::ViewElement;
using SurgSim::Math::BoxShape;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::VirtualToolCoupler;

std::shared_ptr<SceneElement> createPlane(const std::string& name)
{
	std::shared_ptr<DoubleSidedPlaneShape> planeShape = std::make_shared<DoubleSidedPlaneShape>();

	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(name + " Physics");
	physicsRepresentation->setShape(planeShape);

	std::shared_ptr<OsgPlaneRepresentation> graphicsRepresentation =
		std::make_shared<OsgPlaneRepresentation>(name + " Graphics");

	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<OsgProgram> program = std::make_shared<OsgProgram>();

	std::shared_ptr<OsgUniform<Vector4f>> uniform = std::make_shared<OsgUniform<Vector4f>>("color");
	uniform->set(Vector4f(0.0f, 0.6f, 1.0f, 0.0f));
	material->addUniform(uniform);

	program->setFragmentShaderSource(
		"uniform vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_FragColor = color;\n"
		"}");
	material->setProgram(program);
	graphicsRepresentation->setMaterial(material);

	std::shared_ptr<SceneElement> planeElement = std::make_shared<BasicSceneElement>(name);
	planeElement->addComponent(physicsRepresentation);
	planeElement->addComponent(graphicsRepresentation);
	planeElement->addComponent(material);

	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> collisionRepresentation;
	collisionRepresentation = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>(name + " Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	planeElement->addComponent(collisionRepresentation);
	return planeElement;
}


std::shared_ptr<SceneElement> createBox(const std::string& name, const std::string& deviceName)
{
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(0.8, 2.0, 0.2); // in meter

	// Physics Components
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + " Physics");
	physicsRepresentation->setIsGravityEnabled(false);
	physicsRepresentation->setDensity(0.1);
	physicsRepresentation->setShape(box);

	// Collision Components
	auto collisionRepresentation =
		std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Box Collision Representation");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphics Components
	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> graphicsRepresentation =
		std::make_shared<OsgBoxRepresentation>(name + " Graphics");
	graphicsRepresentation->setSizeXYZ(box->getSizeX(), box->getSizeY(), box->getSizeZ());

	// Input Components
	std::shared_ptr<SurgSim::Input::InputComponent> inputComponent =
		std::make_shared<SurgSim::Input::InputComponent>(name + " Input");
	inputComponent->setDeviceName(deviceName);

	// Output Components
	std::shared_ptr<SurgSim::Input::OutputComponent> outputComponent = nullptr;
	outputComponent = std::make_shared<SurgSim::Input::OutputComponent>(name + " Output");
	outputComponent->setDeviceName(deviceName);

	// A virtual tool coupler can be used to connect an input/output device with the physics thread.
	// The physics representation follows the pose provided by the device, and the representation's collisions
	// generate forces and torques on the device.
	std::shared_ptr<VirtualToolCoupler> inputCoupler = std::make_shared<VirtualToolCoupler>(name + " Input Coupler");
	inputCoupler->setInput(inputComponent);
	inputCoupler->setOutput(outputComponent);
	inputCoupler->setRepresentation(physicsRepresentation);
	inputCoupler->setHapticOutputOnlyWhenColliding(true);

	// The SceneElement
	std::shared_ptr<BasicSceneElement> boxElement = std::make_shared<BasicSceneElement>(name);
	boxElement->addComponent(physicsRepresentation);
	boxElement->addComponent(collisionRepresentation);
	boxElement->addComponent(graphicsRepresentation);
	boxElement->addComponent(inputComponent);
	boxElement->addComponent(outputComponent);
	boxElement->addComponent(inputCoupler);

	return boxElement;
}

std::shared_ptr<SceneElement> createBoxForRawInput(const std::string& name, const std::string& deviceName)
{
	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgBoxRepresentation>(name + " Graphics");
	graphicsRepresentation->setSizeXYZ(0.8, 2.0, 0.2);
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<OsgProgram> program = std::make_shared<OsgProgram>();
	program->setVertexShaderSource(
		"void main(void)\n"
		"{\n"
		"    gl_Position = ftransform();\n"
		"}");
	program->setFragmentShaderSource(
		"void main(void)\n"
		"{\n"
		"    gl_FragColor = vec4(0.2, 0.2, 0.2, 1.0);\n"
		"}");
	material->setProgram(program);
	graphicsRepresentation->setMaterial(material);

	std::shared_ptr<SurgSim::Input::InputComponent> inputComponent;
	inputComponent = std::make_shared<SurgSim::Input::InputComponent>(name + " Input");
	inputComponent->setDeviceName(deviceName);

	std::shared_ptr<DriveElementFromInputBehavior> driver;
	driver = std::make_shared<DriveElementFromInputBehavior>(name + " Driver");
	driver->setSource(inputComponent);

	std::shared_ptr<BasicSceneElement> element = std::make_shared<BasicSceneElement>(name);
	element->addComponent(graphicsRepresentation);
	element->addComponent(inputComponent);
	element->addComponent(driver);
	element->addComponent(material);

	return element;
}

int main(int argc, char* argv[])
{
	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	std::shared_ptr<SurgSim::Input::InputManager> inputManager = std::make_shared<SurgSim::Input::InputManager>();

	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);

	const std::string deviceName = "Tool_Device";
	std::shared_ptr<SurgSim::Input::DeviceInterface> device =
		SurgSim::Devices::loadDevice("Device.yaml");
	SURGSIM_ASSERT(device != nullptr) << "Failed to initialize any device.";
	SURGSIM_LOG_IF(std::dynamic_pointer_cast<SurgSim::Devices::IdentityPoseDevice>(device) != nullptr,
		SurgSim::Framework::Logger::getDefaultLogger(), WARNING) << "The InputVtc example was unable to initialize " <<
		"an input device that provides poses, so it will use a constant pose.";
	inputManager->addDevice(device);

	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	scene->addSceneElement(createBox("VTC Box", deviceName));
	scene->addSceneElement(createBoxForRawInput("Raw Input", deviceName));

	std::shared_ptr<SceneElement> plane =  createPlane("Plane");
	plane->setPose(makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, -1.0, 0.0)));
	scene->addSceneElement(plane);

	std::shared_ptr<ViewElement> viewElement = std::make_shared<OsgViewElement>("view");
	viewElement->setPose(makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));
	scene->addSceneElement(viewElement);

	runtime->execute();

	return 0;
}
