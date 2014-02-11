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
#include <string>

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"

/// Load scenery object from file
/// \param name Name of this scenery representation.
/// \param fileName Name of the file from which the scenery representation is loaded.
/// \return A SceneElement containing the scenery representation.
std::shared_ptr<SurgSim::Framework::SceneElement> loadSceneryObject(const std::string& name,
																	const std::string& fileName)
{
	auto sceneryRepresentation = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>("Stapler");
	sceneryRepresentation->setFileName(fileName);

	auto sceneElement =	std::make_shared<SurgSim::Blocks::BasicSceneElement>(name);
	sceneElement->addComponent(sceneryRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView()
{
	auto view = std::make_shared<SurgSim::Graphics::OsgViewElement>("StaplingDemoView");

	view->enableManipulator(true);
	view->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 0.5, 0.5), SurgSim::Math::Vector3d::Zero());

	return view;
}

int main(int argc, char* argv[])
{
	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();
	scene->addSceneElement(loadSceneryObject("stapler", "stapler_collision.obj"));
	scene->addSceneElement(loadSceneryObject("arm", "forearm.osgb"));
	scene->addSceneElement(createView());

	runtime->addManager(graphicsManager);
	runtime->execute();

	return 0;
}
