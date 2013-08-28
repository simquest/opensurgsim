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
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgRenderPass.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/UnitTests/MockCamera.h>
#include <SurgSim/Graphics/OsgRenderTarget.h>
#include <SurgSim/Graphics/OsgMaterial.h>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgRenderPassTests, InitTest)
{

	ASSERT_NO_THROW({OsgRenderPass renderPass("RenderPass");});

	OsgRenderPass renderPass("TestPass");
	EXPECT_NE(nullptr, renderPass.getCamera());
	EXPECT_NE(nullptr, renderPass.getGroup());
	EXPECT_EQ(nullptr, renderPass.getMaterial());
	EXPECT_EQ(nullptr, renderPass.getRenderTarget());

	EXPECT_EQ(renderPass.getGroup().get(), renderPass.getCamera()->getGroup().get());
}

TEST(OsgRenderPassTests, CameraAccessorTest)
{
	auto renderPass = std::make_shared<OsgRenderPass>("RenderPass");
	auto osgCamera = std::make_shared<OsgCamera>("OsgCamera");
	auto mockCamera = std::make_shared<GMockCamera>();

	auto defaultGroup = renderPass->getGroup();

	EXPECT_FALSE(renderPass->setCamera(mockCamera));
	EXPECT_TRUE(renderPass->setCamera(osgCamera));
	
	EXPECT_EQ(osgCamera.get(), renderPass->getCamera().get());
	EXPECT_EQ(defaultGroup.get(), osgCamera->getGroup().get());

}

TEST(OsgRenderPassTests, GroupAccessorTest)
{
	auto renderPass = std::make_shared<OsgRenderPass>("RenderPass");
	auto osgGroup = std::make_shared<OsgGroup>("OsgGroup");
	auto mockGroup = std::make_shared<GMockGroup>();

	auto defaultGroup = renderPass->getGroup();

	EXPECT_FALSE(renderPass->setGroup(mockGroup));
	EXPECT_TRUE(renderPass->setGroup(osgGroup));

	EXPECT_EQ(osgGroup.get(), renderPass->getGroup().get());
	EXPECT_EQ(osgGroup.get(), renderPass->getCamera()->getGroup().get());
}

TEST(OsgRenderPassTests, RenderTargetAccessorTest)
{
	auto renderPass = std::make_shared<OsgRenderPass>("RenderPass");
	auto renderTarget = std::make_shared<OsgRenderTarget2d>(128, 128, 1.0,1, true);

	EXPECT_TRUE(renderPass->setRenderTarget(renderTarget));
	EXPECT_EQ(renderTarget.get(), renderPass->getRenderTarget().get());
	std::shared_ptr<OsgCamera> camera = std::dynamic_pointer_cast<OsgCamera>(renderPass->getCamera());

	EXPECT_TRUE(camera->getOsgCamera()->isRenderToTextureCamera());

	// Not testing Non-Osg assignment yet

}

}; // namespace Graphics
}; // namespace SurgSim
