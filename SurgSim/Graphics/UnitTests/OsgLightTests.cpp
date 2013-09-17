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

/// \file
///	Basic logic tests for OsgLight

#include <gtest/gtest.h>
#include <SurgSim/Graphics/Light.h>
#include <SurgSim/Graphics/OsgLight.h>
#include <SurgSim/Graphics/OsgGroup.h>
#include <SurgSim/Graphics/OsgConversions.h>

#include <SurgSim/Graphics/UnitTests/MockObjects.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace {
	const double epsilon = 1e-8;
}

namespace SurgSim
{
namespace Graphics
{

class OsgLightTests : public testing::Test
{
public:
	testing::AssertionResult hasUniforms(
		std::shared_ptr<OsgLight> light,
		std::shared_ptr<OsgGroup> group, bool doesHave)
	{
		osg::ref_ptr<osg::Group> osgGroup = group->getOsgGroup();
		osg::ref_ptr<osg::StateSet> stateSet = osgGroup->getStateSet();

		for (auto it = std::begin(light->m_uniforms); it != std::end(light->m_uniforms); ++it)
		{
			osg::ref_ptr<osg::Uniform> uniform = stateSet->getUniform(it->second->getName());
			if (doesHave && uniform == nullptr)
			{
				return testing::AssertionFailure() << "Expected uniform " << it->second->getName() << " not found.";
			}
			else if (!doesHave && uniform != nullptr)
			{
				return testing::AssertionFailure() << "Did not expect uniform " << it->second->getName() << 
					" but found it.";
			}
		}
		return testing::AssertionSuccess();
	}

	osg::ref_ptr<osg::Uniform> getUniform(std::shared_ptr<OsgLight> light, int type)
	{
		return light->m_uniforms[type];
	}
};

TEST_F(OsgLightTests, InitTest)
{
	ASSERT_NO_THROW({auto light = std::make_shared<OsgLight>("TestLight");});

	std::shared_ptr<OsgLight> light = std::make_shared<OsgLight>("TestLight");

	ASSERT_EQ(nullptr, light->getGroup());
}



TEST_F(OsgLightTests, GroupAccessorTest)
{
	std::shared_ptr<OsgLight> light = std::make_shared<OsgLight>("TestLight");
	std::shared_ptr<OsgGroup> group = std::make_shared<OsgGroup>("TestGroup");
	std::shared_ptr<MockGroup> mockGroup = std::make_shared<MockGroup>("MockGroup");

	// Light does not have a default group
	EXPECT_EQ(nullptr, light->getGroup());

	// Assigning a light to a group
	EXPECT_TRUE(light->setGroup(group));
	EXPECT_EQ(group, light->getGroup());

	EXPECT_TRUE(hasUniforms(light, group, true));

	// Light should not take a mock group as a group
	EXPECT_FALSE(light->setGroup(mockGroup));
	EXPECT_EQ(group, light->getGroup());

	// Assigning an empty group should clear the group
	EXPECT_TRUE(light->setGroup(nullptr));
	EXPECT_EQ(nullptr, light->getGroup());

	// Still should not accept a mock group
	EXPECT_FALSE(light->setGroup(mockGroup));
	EXPECT_EQ(nullptr, light->getGroup());

	// Setting the nullptr should also work
	EXPECT_TRUE(light->setGroup(nullptr));
	EXPECT_EQ(nullptr, light->getGroup());

	EXPECT_TRUE(hasUniforms(light, group, false));
}

TEST_F(OsgLightTests, ColorAccessorTests)
{
	std::shared_ptr<OsgLight> light = std::make_shared<OsgLight>("TestLight");
	std::shared_ptr<OsgGroup> group = std::make_shared<OsgGroup>("TestGroup");

	light->setGroup(group);

	// We are using the indices directly from the enum, in a white box way, there are not really
	// any good ways to do this better besides digging for the uniform by name ...
	// make sure all the values are different from each other so not to get false positives
	Vector4d ambient(3.0,4.0,5.0,6.0);
	osg::Vec4f osgColor;
	light->setAmbientColor(ambient);
	EXPECT_TRUE(ambient.isApprox(light->getAmbientColor()));
	getUniform(light,1)->get(osgColor);
	EXPECT_TRUE(ambient.isApprox(fromOsg(osgColor).cast<double>()));

	Vector4d diffuse(1.0,2.0,3.0,4.0);
	light->setDiffuseColor(diffuse);
	EXPECT_TRUE(diffuse.isApprox(light->getDiffuseColor()));
	getUniform(light,2)->get(osgColor);
	EXPECT_TRUE(diffuse.isApprox(fromOsg(osgColor).cast<double>()));

	Vector4d specular(2.0,3.0,4.0,5.0);
	light->setSpecularColor(specular);
	EXPECT_TRUE(specular.isApprox(light->getSpecularColor()));
	getUniform(light,3)->get(osgColor);
	EXPECT_TRUE(specular.isApprox(fromOsg(osgColor).cast<double>()));
}

TEST_F(OsgLightTests, AttenuationAccessorTests)
{
	std::shared_ptr<OsgLight> light = std::make_shared<OsgLight>("TestLight");
	std::shared_ptr<OsgGroup> group = std::make_shared<OsgGroup>("TestGroup");

	light->setGroup(group);
	float osgValue;
	double constantAttenuation = 3.0;
	light->setConstantAttenuation(constantAttenuation);
	EXPECT_NEAR(constantAttenuation, light->getConstantAttenuation(), epsilon);
	getUniform(light,4)->get(osgValue);
	EXPECT_NEAR(constantAttenuation, static_cast<double>(osgValue), epsilon);

	double linearAttenuation = 4.0;
	light->setLinearAttenuation(linearAttenuation);
	EXPECT_NEAR(linearAttenuation , light->getLinearAttenuation(), epsilon);
	getUniform(light,5)->get(osgValue);
	EXPECT_NEAR(linearAttenuation, static_cast<double>(osgValue), epsilon);

	double quadraticAttenuation = 5.0;
	light->setQuadraticAttenuation(quadraticAttenuation);
	EXPECT_NEAR(quadraticAttenuation, light->getQuadraticAttenuation(), epsilon);
	getUniform(light,6)->get(osgValue);
	EXPECT_NEAR(quadraticAttenuation, static_cast<double>(osgValue), epsilon);
}

TEST_F(OsgLightTests, PositionUniformTest)
{
	std::shared_ptr<OsgLight> light = std::make_shared<OsgLight>("TestLight");
	std::shared_ptr<OsgGroup> group = std::make_shared<OsgGroup>("TestGroup");

	light->setGroup(group);

	RigidTransform3d pose = RigidTransform3d::Identity();

	Vector3d translate(5.0,6.0,7.0);

	pose.translate(translate);
	
	light->setPose(pose);

	osg::Vec3f osgTranslate;
	getUniform(light,0)->get(osgTranslate);

	EXPECT_TRUE(translate.isApprox(fromOsg(osgTranslate).cast<double>()));
}


}; // namespace Graphics
}; // namespace SurgSim
