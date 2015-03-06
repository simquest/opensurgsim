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

#ifndef SURGSIM_GRAPHICS_RENDERTESTS_RENDERTEST_H
#define SURGSIM_GRAPHICS_RENDERTESTS_RENDERTEST_H

#include <gtest/gtest.h>
#include <memory>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/ApplicationData.h"

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"

namespace SurgSim
{
namespace Framework
{

class Runtime;
class Scene;

}

namespace Graphics
{

struct RenderTest : public ::testing::Test
{
public:

	virtual void SetUp();

	virtual void TearDown();

	std::shared_ptr<ScreenSpaceQuadRepresentation> makeQuad(
		const std::string& name,
		int width,
		int height,
		int x,
		int y);

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<OsgManager> graphicsManager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<OsgViewElement> viewElement;
	std::shared_ptr<const SurgSim::Framework::ApplicationData> applicationData;
	std::shared_ptr<OsgCamera> camera;

};

}; // Graphics
}; // SurgSim

#endif