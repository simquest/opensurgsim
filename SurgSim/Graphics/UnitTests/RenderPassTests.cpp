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


#include <gtest/gtest.h>
#include "SurgSim/Graphics/RenderPass.h"

#include "SurgSim/Graphics/Group.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgMaterial.h"

namespace SurgSim
{
namespace Graphics
{

TEST(RenderPassTests, InitTest)
{
	std::shared_ptr<RenderPass> renderPass;

	ASSERT_NO_THROW({renderPass = std::make_shared<RenderPass>("testpass");});

	EXPECT_NE(nullptr, renderPass->getCamera());
	EXPECT_NE(nullptr, renderPass->getMaterial());
	EXPECT_EQ(renderPass->getCamera()->getMaterial(), renderPass->getMaterial());
	auto references = renderPass->getCamera()->getRenderGroupReferences();
	EXPECT_TRUE(std::find(references.begin(), references.end(), renderPass->getName()) != references.end());

}

TEST(RenderPassTests, SettersTest)
{
	auto renderPass = std::make_shared<RenderPass>("testpass");

	auto renderTarget = std::make_shared<OsgRenderTarget2d>(1024, 1024, 1.0, 1, true);

	ASSERT_NO_THROW({renderPass->setRenderTarget(renderTarget);});
	EXPECT_EQ(renderTarget, renderPass->getCamera()->getRenderTarget());

	auto material = std::make_shared<OsgMaterial>("material");

	ASSERT_NO_THROW({renderPass->setMaterial(material);});
	EXPECT_EQ(material, renderPass->getMaterial());
}

}; // namespace Graphics
}; // namespace SurgSim
