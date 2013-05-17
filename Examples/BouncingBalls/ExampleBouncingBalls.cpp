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

#include "Sphere.h"

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Framework::SceneElement;

int main(int argc, char* argv[])
{
	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime("config.txt"));

	// std::shared_ptr<GraphicsManager> renderManager(new GraphicsManager());
	// std::shared_ptr<ExternalPhysicsManager> physicsManager(new ExternalPhysicsManager());
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager(new SurgSim::Framework::BehaviorManager());

	// runtime->addManager(renderManager);
	// runtime->addManager(physicsManager);
	runtime->addManager(behaviorManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());

	std::shared_ptr<SurgSim::Framework::SceneElement> sphere = std::make_shared<Sphere>("sphere1");
	scene->addSceneElement(sphere);

	runtime->setScene(scene);

	runtime->execute();

	return 0;
}
