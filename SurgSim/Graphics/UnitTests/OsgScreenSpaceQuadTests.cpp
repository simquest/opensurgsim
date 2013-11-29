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

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

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

TEST(OsgScreenSpaceQuadRepresentation, SetSize)
{
	std::shared_ptr<OsgView> view = std::make_shared<OsgView>("view");
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("quad", view);

	double width = 100.0;
	double height = 100.0;

	ASSERT_ANY_THROW(quad->getSize(nullptr, &height));
	ASSERT_ANY_THROW(quad->getSize(&width, nullptr));
	ASSERT_ANY_THROW(quad->getSize(nullptr, nullptr));

	quad->getLocation(&width, &height);
	EXPECT_DOUBLE_EQ(0.0, width);
	EXPECT_DOUBLE_EQ(0.0, height);

	quad->setSize(100.0,200.0);
	quad->getSize(&width, &height);

	EXPECT_DOUBLE_EQ(100.0, width);
	EXPECT_DOUBLE_EQ(200.0, height);

}

TEST(OsgScreenSpaceQuadRepresentationTests, SetLocation)
{
	std::shared_ptr<OsgView> view = std::make_shared<OsgView>("view");
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad =
		std::make_shared<OsgScreenSpaceQuadRepresentation>("quad", view);

	double x = 100.0;
	double y = 100.0;

	ASSERT_ANY_THROW(quad->getLocation(nullptr, &y));
	ASSERT_ANY_THROW(quad->getLocation(&x, nullptr));
	ASSERT_ANY_THROW(quad->getLocation(nullptr, nullptr));

	quad->getLocation(&x, &y);
	EXPECT_DOUBLE_EQ(0.0, x);
	EXPECT_DOUBLE_EQ(0.0, y);

	quad->setLocation(100.0,200.0);
	quad->getLocation(&x, &y);

	EXPECT_DOUBLE_EQ(100.0, x);
	EXPECT_DOUBLE_EQ(200.0, y);

	Vector3d position(300.0, 400.0, 0.0);
	quad->setPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), position));

	quad->getLocation(&x,&y);
	EXPECT_DOUBLE_EQ(300.0, x);
	EXPECT_DOUBLE_EQ(400.0, y);

}


}  // namespace Graphics
}  // namespace SurgSim