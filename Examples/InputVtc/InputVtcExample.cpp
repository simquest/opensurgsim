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

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/Blocks/TransferInputPoseBehavior.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgPlaneRepresentation.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/DoubleSidedPlaneShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

#include "Examples/InputVtc/DeviceFactory.h"

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::Blocks::TransferInputPoseBehavior;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgShader;
using SurgSim::Graphics::OsgUniform;
using SurgSim::Math::BoxShape;
using SurgSim::Math::DoubleSidedPlaneShape;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Physics::VirtualToolCoupler;
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
	std::shared_ptr<DoubleSidedPlaneShape> planeShape = std::make_shared<DoubleSidedPlaneShape>();

	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(name + " Physics");
	RigidRepresentationParameters params;
	params.setShapeUsedForMassInertia(planeShape);
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<OsgPlaneRepresentation> graphicsRepresentation =
		std::make_shared<OsgPlaneRepresentation>(name + " Graphics");
	graphicsRepresentation->setInitialPose(pose);

	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
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

	std::shared_ptr<TransferPoseBehavior> transferPose =
		std::make_shared<TransferPoseBehavior>("Physics to Graphics Pose");
	transferPose->setPoseSender(physicsRepresentation);
	transferPose->setPoseReceiver(graphicsRepresentation);
	planeElement->addComponent(transferPose);
	auto collisionRepresentation =
		std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Plane Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);
	planeElement->addComponent(collisionRepresentation);
	return planeElement;
}


std::shared_ptr<SceneElement> createBox(const std::string& name, const std::string& toolDeviceName)
{
	// Physics Components
	RigidRepresentationParameters params;
	// The VTC needs a mass for the calculations in Physics, but ideally it would track the input perfectly if no
	// collisions occurred.  (Since the VTC follows the input device, it may appear to have mass based on the force
	// output -> pose input feedback, but any inertial forces should be applied directly to the device.)
	// To minimize the VTC's inertia, we set its density (and thereby its mass) low.  However, the lower the
	// Representation's mass (and the longer the inter-update duration), the more likely that high spring forces in the
	// simulation will make the system unstable, creating unbounded oscillations that eventually make the
	// RigidRepresentation invalid.  Therefore, whenever the mass or shape is changed, the inputCoupler's stiffness and
	// damping (translational and angular) should be adjusted to make the system close to critically damped in free
	// motion (i.e., converges quickly without oscillation when no collisions are occurring).
	params.setDensity(0.1);
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(0.8, 2.0, 0.2); // in m
	params.setShapeUsedForMassInertia(box);

	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + "-Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setIsGravityEnabled(false);

	auto collisionRepresentation =
		std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Box Collision Representation");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	// Graphics Components
	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> graphicsRepresentation =
		std::make_shared<OsgBoxRepresentation>(name + "-Graphics");
	graphicsRepresentation->setSize(box->getSizeX(), box->getSizeY(), box->getSizeZ());

	std::shared_ptr<SurgSim::Graphics::BoxRepresentation> rawInputGraphicsRepresentation =
		std::make_shared<OsgBoxRepresentation>(name + "-RawInput-Graphics");
	rawInputGraphicsRepresentation->setSize(box->getSizeX(), box->getSizeY(), box->getSizeZ());
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
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
	rawInputGraphicsRepresentation->setMaterial(material);

	std::shared_ptr<TransferPoseBehavior> transferPose = std::make_shared<TransferPoseBehavior>("Physics to Graphics");
	transferPose->setPoseSender(physicsRepresentation);
	transferPose->setPoseReceiver(graphicsRepresentation);

	// Input Components
	std::shared_ptr<SurgSim::Input::InputComponent> inputComponent =
		std::make_shared<SurgSim::Input::InputComponent>("input component");
	inputComponent->setDeviceName(toolDeviceName);

	std::shared_ptr<SurgSim::Input::OutputComponent> outputComponent = nullptr;
	outputComponent = std::make_shared<SurgSim::Input::OutputComponent>("output component");
	outputComponent->setDeviceName(toolDeviceName);

	std::shared_ptr<TransferInputPoseBehavior> transferInputPose =
		std::make_shared<TransferInputPoseBehavior>("Input to RawInputGraphics");
	transferInputPose->setPoseSender(inputComponent);
	transferInputPose->setPoseReceiver(rawInputGraphicsRepresentation);

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
	std::shared_ptr<VirtualToolCoupler> inputCoupler = std::make_shared<VirtualToolCoupler>("Input Coupler");
	inputCoupler->setInput(inputComponent);
	inputCoupler->setOutput(outputComponent);
	inputCoupler->setRepresentation(physicsRepresentation);
	inputCoupler->setLinearStiffness(params.getMass() * 1000);
	inputCoupler->setLinearDamping(params.getMass() * 25);
	inputCoupler->setAngularStiffness(params.getMass() * 250);
	inputCoupler->setAngularDamping(params.getMass() * 10);
	inputCoupler->setOutputForceScaling(1.0);

	// The SceneElement
	std::shared_ptr<BasicSceneElement> boxElement = std::make_shared<BasicSceneElement>(name);
	boxElement->addComponent(physicsRepresentation);
	boxElement->addComponent(collisionRepresentation);
	boxElement->addComponent(graphicsRepresentation);
	boxElement->addComponent(rawInputGraphicsRepresentation);
	boxElement->addComponent(transferPose);
	boxElement->addComponent(inputComponent);
	boxElement->addComponent(outputComponent);
	boxElement->addComponent(transferInputPose);
	boxElement->addComponent(inputCoupler);

	return boxElement;
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
	scene->addSceneElement(createBox("box", toolDeviceName));
	scene->addSceneElement(createPlane("plane",
				SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, -1.0, 0.0))));
	scene->addSceneElement(createView("view", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	runtime->execute();

	return 0;
}
