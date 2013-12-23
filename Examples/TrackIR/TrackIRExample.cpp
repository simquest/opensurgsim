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

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferInputPoseBehavior.h"
#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgPlaneRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"


using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferInputPoseBehavior;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgShader;
using SurgSim::Graphics::OsgUniform;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;



std::shared_ptr<SurgSim::Graphics::ViewElement> createView(
	const std::string& name, int x, int y, int width, int height,
	std::shared_ptr<SurgSim::Graphics::OsgCamera> camera)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	auto inputComponent = std::make_shared<SurgSim::Input::InputComponent>("input", "TrackIRDevice");
	viewElement->addComponent(inputComponent);
	viewElement->addComponent(std::make_shared<TransferInputPoseBehavior>("TrackIR-Pose", inputComponent, camera));

	return viewElement;
}

std::shared_ptr<SceneElement> createPlane(const std::string& name,
										  const SurgSim::Math::RigidTransform3d& pose)
{
	std::shared_ptr<OsgPlaneRepresentation> graphicsRepresentation =
		std::make_shared<OsgPlaneRepresentation>(name + " Graphics");
	graphicsRepresentation->setInitialPose(pose);

	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
	std::shared_ptr<OsgShader> shader = std::make_shared<OsgShader>();

	std::shared_ptr<OsgUniform<Vector4f>> uniform = std::make_shared<OsgUniform<Vector4f>>("color");
	uniform->set(Vector4f(0.2f, 0.2f, 0.4f, 0.0f));
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
	planeElement->addComponent(graphicsRepresentation);

	return planeElement;
}

std::shared_ptr<SceneElement> createBox(const std::string& name)
{
	auto box1 = std::make_shared<OsgBoxRepresentation>(name + "-Graphics");
	box1->setSize(1.0, 1.0, 1.0);
	box1->setInitialPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(1.5, 1.0, -8.0)));

	auto box2 = std::make_shared<OsgBoxRepresentation>(name + "-Graphics2");
	box2->setSize(1.0, 3.0, 1.0);
	box2->setInitialPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), Vector3d(-1.5, 0.0, -5.0)));

	std::shared_ptr<SceneElement> boxElement = std::make_shared<BasicSceneElement>(name);
	boxElement->addComponent(box1);
	boxElement->addComponent(box2);
	return boxElement;
}


int main(int argc, char* argv[])
{
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	auto inputManager    = std::make_shared<SurgSim::Input::InputManager>();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	std::shared_ptr<SurgSim::Device::TrackIRDevice> toolDevice =
		std::make_shared<SurgSim::Device::TrackIRDevice>("TrackIRDevice");
	SURGSIM_ASSERT( toolDevice->initialize() == true ) <<
		"Could not initialize device '%s' for the tool.\n", toolDevice->getName().c_str();

	inputManager->addDevice(toolDevice);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);
	runtime->addManager(inputManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());
	scene->addSceneElement(createBox("box"));
	scene->addSceneElement(createPlane("plane",
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, -1.0, 0.0))));
	scene->addSceneElement(createView("view", 0, 0, 1024, 768, graphicsManager->getDefaultCamera()));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 100.0)));

	runtime->setScene(scene);
	runtime->execute();

	return 0;
}
