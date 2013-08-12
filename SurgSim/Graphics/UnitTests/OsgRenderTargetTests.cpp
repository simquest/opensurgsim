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

namespace SurgSim
{
namespace Graphics
{


TEST(OsgRenderTargetTests, DefaultConstructorTest)
{
	EXPECT_NO_THROW({OsgPotRenderTarget target;});
	EXPECT_NO_THROW({OsgRectangleRenderTarget target;});
}

TEST(OsgRenderTargetTests, PotDefaultConstructorTest)
{
	EXPECT_NO_THROW({OsgPotRenderTarget target;});
	OsgPotRenderTarget target;

	int width, height;
	target.getSize(&width, &height);
	EXPECT_EQ(0.0, width);
	EXPECT_EQ(0.0, height);
	EXPECT_EQ(1.0, target.getScale());

	EXPECT_FALSE(target.doesUseDepthTarget());
	EXPECT_TRUE(nullptr == target.getDepthTarget());

	EXPECT_FALSE(target.doesUseStencilTarget());
	EXPECT_TRUE(nullptr == target.getStencilTarget());

	ASSERT_EQ(0, target.getColorTargetCount());
	for(int i=0;i<16;++i)
	{
		SCOPED_TRACE("Color Buffer" + std::to_string(i));
		EXPECT_TRUE(nullptr == target.getColorTarget(i));
	}

}


TEST(OsgRenderTargetTests, PotSpecificConstructorTest)
{
	ASSERT_NO_THROW({OsgPotRenderTarget target(256,256,1.0,16,true,true);});

	OsgPotRenderTarget target(256,128,1.0,16,true,true);

	int width, height;
	target.getSize(&width, &height);
	EXPECT_EQ(256, width);
	EXPECT_EQ(128, height);
	EXPECT_EQ(1.0, target.getScale());

	EXPECT_TRUE(target.doesUseDepthTarget());
	EXPECT_FALSE(nullptr == target.getDepthTarget());

	EXPECT_TRUE(target.doesUseStencilTarget());
	EXPECT_FALSE(nullptr == target.getStencilTarget());

	EXPECT_EQ(16, target.getColorTargetCount());
	for(int i=0;i<16;++i)
	{
		SCOPED_TRACE("Color Buffer " + std::to_string(i));
		EXPECT_FALSE(nullptr == target.getColorTarget(i));
	}
}

}; // namespace Graphics
}; // namespace SurgSim
