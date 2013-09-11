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
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgPlaneRepresentation.h>
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgUniform.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Input/InputManager.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>
#include <SurgSim/Physics/BoxShape.h>
#include <SurgSim/Collision/RigidCollisionRepresentation.h>
#include <SurgSim/Collision/RigidShapeCollisionRepresentation.h>
#include <SurgSim/Physics/VtcRigidParameters.h>
#include <SurgSim/Physics/VtcRigidRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::RepresentationPoseBehavior;
using SurgSim::Blocks::VtcToGraphicsPoseBehavior;
using SurgSim::Blocks::InputVtcBehavior;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgShader;
using SurgSim::Graphics::OsgUniform;
using SurgSim::Math::Vector4f;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::Representation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::BoxShape;
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

	std::shared_ptr<SurgSim::Physics::DoubleSidedPlaneShape> planeShape =
		std::make_shared<SurgSim::Physics::DoubleSidedPlaneShape>();

	std::shared_ptr<SceneElement> planeElement = std::make_shared<BasicSceneElement>(name);
	planeElement->addComponent(physicsRepresentation);
	planeElement->addComponent(graphicsRepresentation);

	planeElement->addComponent(std::make_shared<RepresentationPoseBehavior>("Physics to Graphics Pose",
		physicsRepresentation, graphicsRepresentation));
	planeElement->addComponent(std::make_shared<SurgSim::Collision::RigidShapeCollisionRepresentation>
		("Plane Collision",planeShape, physicsRepresentation));
	return planeElement;
}


std::shared_ptr<SceneElement> createBox(const std::string& name)
{
	double density = 700.0;  // Wood in Kg.m-3
	double size[3] = {0.8, 2.0, 0.2};
	double volume = size[0] * size[1] * size[2];
	double mass = volume * density;

	RigidRepresentationParameters params;
	params.setDensity(density);
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(size[0], size[1], size[2]);
	params.setShapeUsedForMassInertia(box);

	VtcRigidParameters vtcParams;
	vtcParams.setVtcAngularDamping(mass * 20);
	vtcParams.setVtcAngularStiffness(mass * 50);
	vtcParams.setVtcLinearDamping(mass * 20);
	vtcParams.setVtcLinearStiffness(mass * 50);

	std::shared_ptr<VtcRigidRepresentation> vtcRepresentation =
		std::make_shared<VtcRigidRepresentation>(name + "-Vtc");
	vtcRepresentation->setInitialParameters(params);
	vtcRepresentation->setInitialVtcParameters(vtcParams);

	std::shared_ptr<OsgBoxRepresentation> graphicsRepresentation =
		std::make_shared<OsgBoxRepresentation>(name + "-Graphics");
	graphicsRepresentation->setSize(box->getSizeX(), box->getSizeY(), box->getSizeZ());

	std::shared_ptr<OsgBoxRepresentation> graphicsRepresentation2 =
		std::make_shared<OsgBoxRepresentation>(name + "2-Graphics");
	graphicsRepresentation2->setSize(box->getSizeX(), box->getSizeY(), box->getSizeZ());

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
	graphicsRepresentation2->setMaterial(material);

	std::shared_ptr<SurgSim::Input::InputComponent> inputComponent =
		std::make_shared<SurgSim::Input::InputComponent>("input", "MultiAxisDevice");

	std::shared_ptr<SceneElement> boxElement = std::make_shared<BasicSceneElement>(name);
	boxElement->addComponent(vtcRepresentation);
	boxElement->addComponent(graphicsRepresentation);
	boxElement->addComponent(graphicsRepresentation2);
	boxElement->addComponent(inputComponent);

	boxElement->addComponent(std::make_shared<InputVtcBehavior>("Input to Vtc",
								inputComponent, vtcRepresentation));
	boxElement->addComponent(std::make_shared<RepresentationPoseBehavior>("Physics to Graphics Pose",
								vtcRepresentation, graphicsRepresentation));
	boxElement->addComponent(std::make_shared<VtcToGraphicsPoseBehavior>("Physics to Graphics2 Pose",
								vtcRepresentation, graphicsRepresentation2));
	boxElement->addComponent(std::make_shared<SurgSim::Collision::RigidCollisionRepresentation>
								("Box Collision Representation", vtcRepresentation));
	return boxElement;
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
	toolDevice->setPositionScale(toolDevice->getPositionScale() * 10.0);
	toolDevice->setOrientationScale(toolDevice->getOrientationScale() * 3.0);
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
	scene->addSceneElement(createBox("box"));
	scene->addSceneElement(createPlane("plane",
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, -1.0, 0.0))));
	scene->addSceneElement(createView("view", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	runtime->setScene(scene);
	runtime->execute();

	return 0;
}
