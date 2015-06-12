// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Tests for the OsgTexture class.

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgTexture.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <osg/Texture2D>

#include <gtest/gtest.h>

#include <sstream>

namespace SurgSim
{
namespace Graphics
{

/// Concrete OSG Texture for testing.
/// Wraps an osg::Texture2D.
class MockOsgTexture : public OsgTexture
{
public:
	/// Constructor
	MockOsgTexture() : OsgTexture(new osg::Texture2D())
	{
	}
};

TEST(OsgTextureTests, InitTest)
{
	MockOsgTexture texture;

	EXPECT_NE(nullptr, texture.getOsgTexture());

	EXPECT_EQ(nullptr, texture.getOsgTexture()->getImage(0u));
}

TEST(OsgTextureTests, LoadAndClearImageTest)
{
	SurgSim::Framework::ApplicationData data("config.txt");

	std::string imagePath = data.findFile("Textures/CheckerBoard.png");

	ASSERT_NE("", imagePath) << "Could not find image file!";

	std::shared_ptr<OsgTexture> osgTexture = std::make_shared<MockOsgTexture>();
	std::shared_ptr<Texture> texture = osgTexture;

	EXPECT_TRUE(texture->loadImage(imagePath)) << "Failed to load image!";

	EXPECT_NE(nullptr, osgTexture->getOsgTexture()->getImage(0u)) << "Texture should have an image!";

	texture->clearImage();

	EXPECT_EQ(nullptr, osgTexture->getOsgTexture()->getImage(0u)) << "Texture image should have been cleared!";
}

}  // namespace Graphics
}  // namespace SurgSim
