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

#include "Examples/ExampleStapling/StaplerBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/VisualizeContactsBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Input::InputManager;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::PhysicsManager;

template <typename Type>
std::shared_ptr<Type> getComponentChecked(std::shared_ptr<SurgSim::Framework::SceneElement> sceneElement,
										  const std::string& name)
{
	std::shared_ptr<SurgSim::Framework::Component> component = sceneElement->getComponent(name);
	SURGSIM_ASSERT(component != nullptr) << "Failed to get Component named '" << name << "'.";

	std::shared_ptr<Type> result = std::dynamic_pointer_cast<Type>(component);
	SURGSIM_ASSERT(result != nullptr) << "Failed to convert Component to requested type.";

	return result;
}

int main(int argc, char* argv[])
{
	{
		StaplerBehavior temporaryStaplerBehavior("TemporaryStaplerBehavior");
		SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior temporaryBehavior("TemporaryBehavior");
		SurgSim::Blocks::VisualizeContactsBehavior visualizeContactsBehavior("TemporaryVisualizeContactsBehavior");
		SurgSim::Graphics::OsgMeshRepresentation temporaryOsgMesh("TemporaryOsgMesh");
		SurgSim::Graphics::OsgSceneryRepresentation temporaryOsgScenery("TemporaryOsgScenery");
		SurgSim::Physics::Fem3DRepresentation temporaryFem3D("TemporaryFem3D");
		SurgSim::Physics::FixedRepresentation temporaryFixedRepresentation("TemporaryFixedRepresentation");
		SurgSim::Physics::VirtualToolCoupler temporaryVirtualToolCoupler("TemporaryVirtualToolCoupler");
	}

	const std::string deviceName = "MultiAxisDevice";

	std::shared_ptr<BehaviorManager> behaviorManager = std::make_shared<BehaviorManager>();
	std::shared_ptr<OsgManager> graphicsManager = std::make_shared<OsgManager>();
	std::shared_ptr<InputManager> inputManager = std::make_shared<InputManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>("config.txt");
	runtime->addManager(behaviorManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(inputManager);
	runtime->addManager(physicsManager);

	std::shared_ptr<SurgSim::Input::DeviceInterface> device;
	device = std::make_shared<SurgSim::Device::IdentityPoseDevice>(deviceName);
	inputManager->addDevice(device);

	std::shared_ptr<OsgViewElement> view = std::make_shared<OsgViewElement>("StaplingDemoView");
	view->enableManipulator(true);
	view->setManipulatorParameters(Vector3d(0.0, 0.5, 0.5), Vector3d::Zero());
	view->enableKeyboardDevice(true);
	inputManager->addDevice(view->getKeyboardDevice());

	YAML::Node node = YAML::LoadFile("Data/Stapling/StaplingDemo.yaml");

	runtime->getScene()->decode(node);
	runtime->getScene()->addSceneElement(view);

	std::shared_ptr<SceneElement> arm;
	std::shared_ptr<SceneElement> wound;
	auto sceneElements = runtime->getScene()->getSceneElements();
	for (auto it = std::begin(sceneElements); it != std::end(sceneElements); ++it)
	{
		if ((*it)->getName() == "armSceneElement")
		{
			arm = (*it);
		}
		if ((*it)->getName() == "wound")
		{
			wound = (*it);
		}
	}
	physicsManager->addExcludedCollisionPair(
		getComponentChecked<SurgSim::Collision::Representation>(wound, "Collision"),
		getComponentChecked<SurgSim::Collision::Representation>(arm, "Collision"));

	runtime->execute();

	return 0;
}