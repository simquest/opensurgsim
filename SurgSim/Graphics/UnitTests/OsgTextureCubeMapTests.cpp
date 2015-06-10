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
/// Tests for the OsgTextureCubeMap class.

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgTextureCubeMap.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <gtest/gtest.h>

#include <sstream>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgTextureCubeMapTests, InitTest)
{
	OsgTextureCubeMap texture;

	EXPECT_NE(nullptr, texture.getOsgTexture());

	EXPECT_EQ(nullptr, texture.getOsgTexture()->getImage(0u));
}

TEST(OsgTextureCubeMapTests, SetSizeTest)
{
	OsgTextureCubeMap texture;

	texture.setSize(256, 512);

	int width, height;
	texture.getSize(&width, &height);

	EXPECT_EQ(256, width);
	EXPECT_EQ(512, height);
}

TEST(OsgTextureCubeMapTests, LoadAndClearImageTest)
{
	SurgSim::Framework::ApplicationData data("config.txt");

	std::string imagePath = data.findFile("Textures/CubeMap_axes.png");

	ASSERT_NE("", imagePath) << "Could not find image file!";

	// Load the image
	std::shared_ptr<OsgTextureCubeMap> osgTexture = std::make_shared<OsgTextureCubeMap>();
	std::shared_ptr<Texture> texture = osgTexture;

	EXPECT_TRUE(texture->loadImage(imagePath)) << "Failed to load image!";

	EXPECT_EQ(6u, osgTexture->getOsgTexture()->getNumImages());

	// Make sure each face has the expected size
	int width, height;
	osgTexture->getSize(&width, &height);
	EXPECT_EQ(256, width);
	EXPECT_EQ(256, height);

	for (size_t i = 0; i < 6; ++i)
	{
		osg::Image* image = osgTexture->getOsgTexture()->getImage(i);
		ASSERT_NE(nullptr, image) << "The texture should have an image for each face!";
		EXPECT_EQ(256, image->s());
		EXPECT_EQ(256, image->t());
		EXPECT_EQ(1, image->r());
	}

	// Remove the image
	texture->clearImage();

	EXPECT_EQ(nullptr, osgTexture->getOsgTexture()->getImage(0u)) << "Texture image should have been cleared!";

	// Try to load an image that does not exist
	EXPECT_FALSE(texture->loadImage("NotHere.png")) << "Should not have been able to load image - it does not exist!";
	EXPECT_EQ(nullptr, osgTexture->getOsgTexture()->getImage(0u));
}

TEST(OsgTextureCubeMapTests, LoadImageFacesTest)
{
	SurgSim::Framework::ApplicationData data("config.txt");

	std::string negativeXPath = data.findFile("Textures/NegativeX.png");
	ASSERT_NE("", negativeXPath) << "Could not find image file for (-X) face!";

	std::string positiveXPath = data.findFile("Textures/PositiveX.png");
	ASSERT_NE("", positiveXPath) << "Could not find image file for (+X) face!";

	std::string negativeYPath = data.findFile("Textures/NegativeY.png");
	ASSERT_NE("", negativeYPath) << "Could not find image file for (-Y) face!";

	std::string positiveYPath = data.findFile("Textures/PositiveY.png");
	ASSERT_NE("", positiveYPath) << "Could not find image file for (+Y) face!";

	std::string negativeZPath = data.findFile("Textures/NegativeZ.png");
	ASSERT_NE("", negativeZPath) << "Could not find image file for (-Z) face!";

	std::string positiveZPath = data.findFile("Textures/PositiveZ.png");
	ASSERT_NE("", positiveZPath) << "Could not find image file for (+Z) face!";

	// Load the images
	std::shared_ptr<OsgTextureCubeMap> osgTexture = std::make_shared<OsgTextureCubeMap>();
	std::shared_ptr<Texture> texture = osgTexture;

	EXPECT_TRUE(osgTexture->loadImageFaces(negativeXPath, positiveXPath, negativeYPath, positiveYPath,
		negativeZPath, positiveZPath)) << "Failed to load images!";

	EXPECT_EQ(6u, osgTexture->getOsgTexture()->getNumImages());

	// Make sure each face has the expected size
	int width, height;
	osgTexture->getSize(&width, &height);
	EXPECT_EQ(256, width);
	EXPECT_EQ(256, height);

	for (size_t i = 0; i < 6; ++i)
	{
		osg::Image* image = osgTexture->getOsgTexture()->getImage(i);
		ASSERT_NE(nullptr, image) << "The texture should have an image for each face!";
		EXPECT_EQ(256, image->s());
		EXPECT_EQ(256, image->t());
		EXPECT_EQ(1, image->r());
	}

	// Remove the image
	texture->clearImage();

	EXPECT_EQ(nullptr, osgTexture->getOsgTexture()->getImage(0u)) << "Texture image should have been cleared!";

	// Try to load an image that does not exist
	EXPECT_FALSE(osgTexture->loadImageFaces(negativeXPath, positiveXPath, "NotHere.png", positiveYPath,
		negativeZPath, positiveZPath)) << "Should not have been able to load an image - it does not exist!";
	EXPECT_EQ(nullptr, osgTexture->getOsgTexture()->getImage(0u));
}

}  // namespace Graphics
}  // namespace SurgSim
