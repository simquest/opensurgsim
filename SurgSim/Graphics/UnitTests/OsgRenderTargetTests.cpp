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
#include <SurgSim/Graphics/OsgRenderTarget.h>
#include <SurgSim/Graphics/OsgCamera.h>

namespace SurgSim
{
namespace Graphics
{


TEST(OsgRenderTargetTests, DefaultConstructorTest)
{
	EXPECT_NO_THROW({OsgRenderTarget2d target;});
	EXPECT_NO_THROW({OsgRenderTargetRectangle target;});
}

TEST(OsgRenderTargetTests, PotDefaultConstructorTest)
{
	EXPECT_NO_THROW({OsgRenderTarget2d target;});
	OsgRenderTarget2d target;

	int width, height;
	target.getSize(&width, &height);
	EXPECT_EQ(0.0, width);
	EXPECT_EQ(0.0, height);

	EXPECT_FALSE(target.doesUseDepthTarget());
	EXPECT_TRUE(nullptr == target.getDepthTarget());

	ASSERT_EQ(0, target.getColorTargetCount());
	for(int i=0; i<16; ++i)
	{
		EXPECT_TRUE(nullptr == target.getColorTarget(i)) << "color target should be nullptr at index:" << i;
	}
}


TEST(OsgRenderTargetTests, PotSpecificConstructorTest)
{
	ASSERT_NO_THROW({OsgRenderTarget2d target(256,256,1.0,16,true);});

	OsgRenderTarget2d target(256,128,1.0,16,true);

	int width, height;
	target.getSize(&width, &height);
	EXPECT_EQ(256, width);
	EXPECT_EQ(128, height);

	EXPECT_TRUE(target.doesUseDepthTarget());
	EXPECT_FALSE(nullptr == target.getDepthTarget());

	EXPECT_EQ(16, target.getColorTargetCount());
	for(int i=0; i<16; ++i)
	{
		EXPECT_FALSE(nullptr == target.getColorTarget(i)) << "color target is nullptr at index:" << i;
	}
}

TEST(OsgRenderTargetTests, CameraTest)
{
	auto camera = std::make_shared<OsgCamera>("Camera1");
	auto renderTarget1 = std::make_shared<OsgRenderTarget2d>(256,256,1.0,8,true);
	auto renderTarget2 = std::make_shared<OsgRenderTarget2d>(128,128,1.0,8,true);

	EXPECT_NO_THROW(camera->setRenderTarget(renderTarget1));
	EXPECT_NO_THROW(camera->setRenderTarget(renderTarget2));
}

}; // namespace Graphics
}; // namespace SurgSim
