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

#include "SurgSim/Graphics/RenderTests//RenderTest.h"

#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Graphics
{


void RenderTest::SetUp()
{
	runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	applicationData = runtime->getApplicationData();
	graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

	runtime->addManager(graphicsManager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	scene = runtime->getScene();

	viewElement = std::make_shared<OsgViewElement>("view element");
	std::array<int, 2> position = {100, 100};
	viewElement->getView()->setPosition(position);
	viewElement->getView()->setWindowBorderEnabled(true);

	camera = std::dynamic_pointer_cast<OsgCamera>(viewElement->getCamera());

	scene->addSceneElement(viewElement);
}

void RenderTest::TearDown()
{
	runtime->stop();
}

std::shared_ptr<ScreenSpaceQuadRepresentation> RenderTest::makeQuad(
	const std::string& name,
	int width,
	int height,
	int x,
	int y)
{
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>(name);
	quad->setSize(width, height);
	Quaterniond quat;
	quat = SurgSim::Math::makeRotationQuaternion(0.0, Vector3d::UnitY().eval());
	quad->setLocalPose(SurgSim::Math::makeRigidTransform(quat, Vector3d(x, y, -0.2)));
	return quad;
}

}; // Graphics
}; // SurgSim
