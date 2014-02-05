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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Graphics/View.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Graphics
{


void RenderTest::SetUp()
{
	applicationData = std::make_shared<SurgSim::Framework::ApplicationData>("config.txt");
	runtime = std::make_shared<SurgSim::Framework::Runtime>();
	graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

	runtime->addManager(graphicsManager);
	runtime->addManager(std::make_shared<SurgSim::Framework::BehaviorManager>());

	scene = runtime->getScene();

	viewElement = std::make_shared<OsgViewElement>("view element");
	viewElement->getView()->setPosition(100,100);
	viewElement->getView()->setWindowBorderEnabled(true);
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
	quad->setSize(width,height);
	Quaterniond quat;
	quat = SurgSim::Math::makeRotationQuaternion<double,Eigen::DontAlign>(0.0,Vector3d::UnitY());
	quad->setInitialPose(SurgSim::Math::makeRigidTransform(quat, Vector3d(x,y,-0.2)));
	return quad;
}

}; // Graphics
}; // SurgSim
