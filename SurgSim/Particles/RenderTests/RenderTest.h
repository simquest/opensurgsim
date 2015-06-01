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

#ifndef SURGSIM_PARTICLES_RENDERTESTS_RENDERTEST_H
#define SURGSIM_PARTICLES_RENDERTESTS_RENDERTEST_H

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/PhysicsManager.h"

namespace SurgSim
{
namespace Particles
{

struct RenderTests : public ::testing::Test
{
public:

	virtual void SetUp();

	virtual void TearDown();

	virtual void runTest(const SurgSim::Math::Vector3d& cameraPosition,
		const SurgSim::Math::Vector3d& cameraLookAt, double miliseconds);

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager;
	std::shared_ptr<SurgSim::Physics::PhysicsManager> physicsManager;
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<SurgSim::Graphics::OsgViewElement> viewElement;
};

}; // namespace Particles
}; // namespace SurgSim

#endif //SURGSIM_PARTICLES_RENDERTESTS_RENDERTEST_H
