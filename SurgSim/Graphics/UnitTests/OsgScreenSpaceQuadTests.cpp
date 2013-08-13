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

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Graphics/OsgTexture.h>
#include <SurgSim/Graphics/OsgUniform.h>
#include <SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgView.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <osg/Texture2D>
#include <osg/StateSet>

#include <gtest/gtest.h>

#include <sstream>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgScreenSpaceQuadRepresentationTests, SetTexture2d)
{
	std::shared_ptr<OsgView> view = std::make_shared<OsgView>("view");
	std::shared_ptr<OsgTexture2d> texture = std::make_shared<OsgTexture2d>();
	texture->setSize(256,256);
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("quad", view);

	EXPECT_TRUE(quad->setTexture(texture));

	std::shared_ptr<OsgMaterial> osgMaterial = std::dynamic_pointer_cast<OsgMaterial>(quad->getMaterial());

	ASSERT_TRUE(osgMaterial != nullptr);
	osg::StateSet* stateSet = osgMaterial->getOsgStateSet();
	EXPECT_EQ(1u, stateSet->getTextureAttributeList().size());
	osg::Texture* osgTexture =
		dynamic_cast<osg::Texture*>(stateSet->getTextureAttribute(0,osg::StateAttribute::TEXTURE));
	EXPECT_EQ(osgTexture, texture->getOsgTexture().get());


	// Test Replacement
	std::shared_ptr<OsgTexture2d> texture2 = std::make_shared<OsgTexture2d>();
	texture2->setSize(256,256);

	EXPECT_TRUE(quad->setTexture(texture2));
	stateSet = osgMaterial->getOsgStateSet();
	EXPECT_EQ(1u, stateSet->getTextureAttributeList().size());
	osgTexture = dynamic_cast<osg::Texture*>(stateSet->getTextureAttribute(0,osg::StateAttribute::TEXTURE));

	EXPECT_EQ(osgTexture, texture2->getOsgTexture().get());
}

TEST(OsgScreenSpaceQuadRepresentationTests, SetTextureRectangle)
{
	std::shared_ptr<OsgView> view = std::make_shared<OsgView>("view");
	std::shared_ptr<TextureRectangle> texture = std::make_shared<OsgTextureRectangle>();
	texture->setSize(10,100);
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("quad", view);

	EXPECT_TRUE(quad->setTexture(texture));

	std::shared_ptr<OsgMaterial> osgMaterial = std::dynamic_pointer_cast<OsgMaterial>(quad->getMaterial());

	ASSERT_TRUE(osgMaterial != nullptr);
	osg::StateSet* stateSet = osgMaterial->getOsgStateSet();
	EXPECT_EQ(1u, stateSet->getTextureAttributeList().size());
}



}  // namespace Graphics
}  // namespace SurgSim