// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Blocks/PostprocessingView.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Blocks/PoseInterpolator.h"
#include "../../Framework/BehaviorManager.h"


using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Blocks
{


TEST(PostprocessingViewTest, BasicScene)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	auto manager = std::make_shared<Graphics::OsgManager>();
	runtime->addManager(manager);
	manager->setRate(140);
	runtime->addManager(std::make_shared<Framework::BehaviorManager>());

	auto scene = runtime->getScene();

	auto display = std::make_shared<Blocks::PostprocessingView>("Display");
	//auto display = std::make_shared<Graphics::OsgViewElement>("Display");
	//display->enableManipulator(true);

	auto pose =
		Math::makeRigidTransform(Vector3d(2.0, 2.0, 2.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
	display->setPose(pose);

	scene->addSceneElement(display);

	{

		auto element = std::make_shared<Framework::BasicSceneElement>("Graphics");
		//element->setPose(Math::makeRigidTranslation(Math::Vector3d(0.0, 0.0, 0.0)));
		auto box = std::make_shared<Graphics::OsgBoxRepresentation>("Box");

		RigidTransform3d from =
			Math::makeRigidTransform(Vector3d(0.2, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
		RigidTransform3d to =
			Math::makeRigidTransform(Vector3d(-0.2, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
		auto interpolator = std::make_shared<SurgSim::Blocks::PoseInterpolator>("interpolator");

		interpolator->setDuration(2.0);
		interpolator->setStartingPose(from);
		interpolator->setEndingPose(to);
		interpolator->setPingPong(true);
		interpolator->setTarget(element);

		element->addComponent(box);
		element->addComponent(interpolator);
		scene->addSceneElement(element);
	}

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(40000));
	manager->dumpDebugInfo();
	runtime->stop();

}

}
}
