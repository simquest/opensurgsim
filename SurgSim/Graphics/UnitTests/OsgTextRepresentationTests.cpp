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
/// Tests for OsgInfo class.

#include <string>

#include <gtest/gtest.h>

#include "SurgSim/Graphics/Graphics.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Graphics/OsgFont.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Graphics
{

TEST(OsgTextRepresentationTests, SetLocation)
{
	auto text = std::make_shared<OsgTextRepresentation>("text");

	double x = 100.0;
	double y = 100.0;

	ASSERT_ANY_THROW(text->getLocation(nullptr, &y));
	ASSERT_ANY_THROW(text->getLocation(&x, nullptr));
	ASSERT_ANY_THROW(text->getLocation(nullptr, nullptr));

	text->getLocation(&x, &y);
	EXPECT_DOUBLE_EQ(0.0, x);
	EXPECT_DOUBLE_EQ(0.0, y);

	text->setLocation(100.0, 200.0);
	text->getLocation(&x, &y);

	EXPECT_DOUBLE_EQ(100.0, x);
	EXPECT_DOUBLE_EQ(200.0, y);

	Vector3d position(300.0, 400.0, 0.0);
	text->setLocalPose(SurgSim::Math::makeRigidTransform(Quaterniond::Identity(), position));
	text->getLocation(&x, &y);
	EXPECT_DOUBLE_EQ(300.0, x);
	EXPECT_DOUBLE_EQ(400.0, y);
}

TEST(OsgTextRepresentationTests, TextTest)
{
	auto text = std::make_shared<OsgTextRepresentation>("text");
	std::string testText("HelloWorld");
	EXPECT_NO_THROW(text->setText(testText));
}

TEST(OsgTextRepresentationTests, FontTests)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto text = std::make_shared<OsgTextRepresentation>("text");

	std::shared_ptr<Font> font = std::make_shared<OsgFont>();
	font->load("Fonts/Vera.ttf");

	// Should have default font
	EXPECT_NE(nullptr, text->getFont());

	// Font setting should work
	EXPECT_NO_THROW(text->setFont(font));
	EXPECT_EQ(font, text->getFont());

	EXPECT_ANY_THROW(text->setFont(nullptr));

	// Other ways to set the font
	EXPECT_NO_THROW(text->loadFont("Fonts/Vera.ttf"));
	EXPECT_NO_THROW(text->setValue("FontFileName", std::string("Fonts/Vera.ttf")));
}

TEST(OsgTextRepresentationTests, MaximumWidth)
{
	auto text = std::make_shared<OsgTextRepresentation>("text");
	SurgSim::DataStructures::OptionalValue<double> optional;
	EXPECT_DOUBLE_EQ(0.0, text->getMaximumWidth());
	EXPECT_EQ(optional, text->getOptionalMaximumWidth());

	text->setMaximumWidth(10.0);
	EXPECT_DOUBLE_EQ(10.0, text->getMaximumWidth());
	optional.setValue(10.0);
	EXPECT_DOUBLE_EQ(*optional, *(text->getOptionalMaximumWidth()));
	optional.invalidate();
	text->setOptionalMaximumWidth(optional);
	EXPECT_DOUBLE_EQ(0.0, text->getMaximumWidth());
	EXPECT_FALSE(text->getOptionalMaximumWidth().hasValue());
	optional.setValue(20.0);
	text->setOptionalMaximumWidth(optional);
	EXPECT_DOUBLE_EQ(20.0, text->getMaximumWidth());
	EXPECT_DOUBLE_EQ(*optional, *(text->getOptionalMaximumWidth()));

}

TEST(OsgTextRepresentationTests, Serialization)
{
	using SurgSim::Framework::Component;
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	std::shared_ptr<TextRepresentation> text = std::make_shared<OsgTextRepresentation>("text");

	text->setText("TestTest");
	text->setFontSize(123.0);
	text->setColor(SurgSim::Math::Vector4d(1.0, 2.0, 3.0, 4.0));
	text->setMaximumWidth(321.0);

	typedef YAML::convert<Component> Converter;

	YAML::Node node;

	ASSERT_NO_THROW(node = Converter::encode(*text));

	std::shared_ptr<Component> result;

	ASSERT_NO_THROW(result = node.as<std::shared_ptr<Component>>());
	ASSERT_NE(nullptr, result);

	auto textResult = std::dynamic_pointer_cast<OsgTextRepresentation>(result);
	ASSERT_NE(nullptr, textResult);

	EXPECT_EQ("TestTest", textResult->getText());
	EXPECT_DOUBLE_EQ(123.0, textResult->getFontSize());
	EXPECT_DOUBLE_EQ(321.0, textResult->getMaximumWidth());
	EXPECT_TRUE(SurgSim::Math::Vector4d(1.0, 2.0, 3.0, 4.0).isApprox(textResult->getColor()));
}

};  // Graphics
}; // SurgSim
