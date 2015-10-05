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

#include "Examples/Stapling/StaplerBehavior.h"

#include "SurgSim/Blocks/Blocks.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/Framework.h"
#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Input/Input.h"
#include "SurgSim/Math/Math.h"
#include "SurgSim/Physics/Physics.h"

using SurgSim::Devices::IdentityPoseDevice;
using SurgSim::Devices::MultiAxisDevice;
using SurgSim::Framework::BehaviorManager;
using SurgSim::Framework::Runtime;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgManager;
using SurgSim::Graphics::OsgView;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Input::InputManager;
using SurgSim::Math::Vector3d;
using SurgSim::Input::DeviceInterface;
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

	std::shared_ptr<DeviceInterface> device;
	device = std::make_shared<MultiAxisDevice>(deviceName);
	if (!device->initialize())
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Could not initialize device " << device->getName() << " for the tool.";

		device = std::make_shared<IdentityPoseDevice>(deviceName);
	}
	inputManager->addDevice(device);

	runtime->loadScene("StaplingDemo.yaml");

	std::shared_ptr<SceneElement> view = runtime->getScene()->getSceneElement("StaplingDemoView");
	auto osgView = std::dynamic_pointer_cast<OsgView>(view->getComponent("StaplingDemoView View"));
	SURGSIM_ASSERT(nullptr != osgView) << "No OsgView held by SceneElement StaplingDemoView.";
	inputManager->addDevice(osgView->getKeyboardDevice());

	runtime->execute();

	return 0;
}
