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
#include "SurgSim/Devices/DeviceFilters/VelocityFromPose.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

#include "Examples/InputVtc/DeviceFactory.h"

using SurgSim::Blocks::DriveElementFromInputBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgShader;
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
	std::shared_ptr<OsgShader> shader = std::make_shared<OsgShader>();

	std::shared_ptr<OsgUniform<Vector4f>> uniform = std::make_shared<OsgUniform<Vector4f>>("color");
	uniform->set(Vector4f(0.0f, 0.6f, 1.0f, 0.0f));
	material->addUniform(uniform);

	shader->setFragmentShaderSource(
		"uniform vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_FragColor = color;\n"
		"}");
	material->setShader(shader);
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


std::shared_ptr<SceneElement> createBox(const std::string& name, const std::string& toolDeviceName)
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
	inputComponent->setDeviceName(toolDeviceName);

	// Output Components
	std::shared_ptr<SurgSim::Input::OutputComponent> outputComponent = nullptr;
	outputComponent = std::make_shared<SurgSim::Input::OutputComponent>(name + " Output");
	outputComponent->setDeviceName(toolDeviceName);

	// A VTC (virtual tool coupler, aka "god object") is used to couple an input/output thread and a physics thread,
	// running at different rates.  Picture a user holding a haptic device (e.g., Falcon or Omni).  The
	// device's pose is used to position a simulated tool, but that pose may cause collisions and the resulting forces
	// are to be displayed to the user via the device.  If the collisions and physics response can be determined in the
	// callback from the device's API, the appropriate forces can be calculated there.  Unfortunately, typically physics
	// threads update much slower than the ~1000 Hz used for threads controlling haptic devices.  For example, if the
	// physics thread updates at 100 Hz, there will be ~10 haptic callbacks that each receive the same force, which
	// tends to create an unstable response in the haptic device (delays in feedback loops often cause limit cycles and
	// other instabilities), and reduces the fidelity of the haptic "feel".
	//
	// Instead, we couple the pose coming from the haptic device to a "virtual tool".  The virtual tool follows the
	// input pose exactly as long as it is not colliding.  As soon as the virtual tool collides, it interacts with the
	// scene normally (through collisions and physics), plus the virtual tool and haptic device are connected via spring
	// & damper forces.
	//
	// The spring forces attempt to pull the haptic device and the virtual tool together (pulling against the user on
	// one side and the physics scene on the other).  The damping forces remove energy from the system to increase
	// stability.  Note that the forces applied to the haptic device come solely from the spring & damper connected to
	// the virtual tool, should be zero when the tool is not colliding, and should be calculated in a high update rate
	// thread.  We pass the device forces&torques and the derivatives (Jacobians) of forces&torques with respect to
	// position and velocity, so that the device's Scaffold can make those calculations.  The forces on the device can
	// be scaled up or down from the forces on the virtual tool.
	std::shared_ptr<VirtualToolCoupler> inputCoupler = std::make_shared<VirtualToolCoupler>(name + " Input Coupler");
	inputCoupler->setInput(inputComponent);
	inputCoupler->setOutput(outputComponent);
	inputCoupler->setRepresentation(physicsRepresentation);

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

std::shared_ptr<SceneElement> createBoxForRawInput(const std::string& name, const std::string& toolDeviceName)
{
	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgBoxRepresentation>(name + " Graphics");
	graphicsRepresentation->setSizeXYZ(0.8, 2.0, 0.2);
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<OsgShader> shader = std::make_shared<OsgShader>();
	shader->setVertexShaderSource(
		"void main(void)\n"
		"{\n"
		"    gl_Position = ftransform();\n"
		"}");
	shader->setFragmentShaderSource(
		"void main(void)\n"
		"{\n"
		"    gl_FragColor = vec4(0.2, 0.2, 0.2, 1.0);\n"
		"}");
	material->setShader(shader);
	graphicsRepresentation->setMaterial(material);

	std::shared_ptr<SurgSim::Input::InputComponent> inputComponent;
	inputComponent = std::make_shared<SurgSim::Input::InputComponent>(name + " Input");
	inputComponent->setDeviceName(toolDeviceName);

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
	static const char* const toolDeviceName = "Tool Device";
	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	std::shared_ptr<SurgSim::Input::InputManager> inputManager = std::make_shared<SurgSim::Input::InputManager>();

	DeviceFactory deviceFactory;
	std::shared_ptr<SurgSim::Input::DeviceInterface> device = deviceFactory.getDevice(toolDeviceName);
	SURGSIM_ASSERT(device != nullptr) << "Unable to get a device, is one connected?";

	inputManager->addDevice(device);

	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());
	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	scene->addSceneElement(createBox("VTC Box", toolDeviceName));
	scene->addSceneElement(createBoxForRawInput("Raw Input", toolDeviceName));

	std::shared_ptr<SceneElement> plane =  createPlane("Plane");
	plane->setPose(makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, -1.0, 0.0)));
	scene->addSceneElement(plane);

	std::shared_ptr<ViewElement> viewElement = std::make_shared<OsgViewElement>("view");
	viewElement->setPose(makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));
	scene->addSceneElement(viewElement);

	runtime->execute();

	return 0;
}
