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

#include <memory>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgTexture1d.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgTexture3d.h"
#include "SurgSim/Graphics/OsgTextureRectangle.h"

#include <osg/StateSet>

namespace
{
std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();
}

namespace SurgSim
{
namespace Graphics
{

TEST(OsgTextureUniformTest, AddUniformTests)
{
	auto uniform2d = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("TextureUniform");

	{
		// Material not initialized, should be able to add Uniform without Texture
		auto material = std::make_shared<OsgMaterial>("Material");
		EXPECT_NO_THROW(material->addUniform(uniform2d));
	}

	{
		// Material is initialized, should not be able to add Uniform without Texture
		auto material = std::make_shared<OsgMaterial>("Material");
		material->initialize(runtime);
		EXPECT_ANY_THROW(material->addUniform(uniform2d));
	}

	{
		// Material is initialized, should be able to add Uniform with Texture
		auto material = std::make_shared<OsgMaterial>("Material");
		material->initialize(runtime);
		auto texture2d = std::make_shared<OsgTexture2d>();
		texture2d->setSize(256, 256);
		uniform2d->set(texture2d);
		EXPECT_NO_THROW(material->addUniform(uniform2d));
	}
}

// Check for correct assignment of uniforms to texture units
TEST(OsgTextureUniformTests, TextureUnitAssignment)
{
	auto material = std::make_shared<OsgMaterial>();
	material->initialize(runtime);
	auto uniform2d0 = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("TextureUniform0");
	auto uniform2d1 = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("TextureUniform1");
	auto uniform2d2 = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("TextureUniform2");
	uniform2d1->setMinimumTextureUnit(8);

	auto texture2d0 = std::make_shared<OsgTexture2d>();
	uniform2d0->set(texture2d0);

	auto texture2d1 = std::make_shared<OsgTexture2d>();
	uniform2d1->set(texture2d1);

	auto texture2d2 = std::make_shared<OsgTexture2d>();
	uniform2d2->set(texture2d2);

	osg::StateSet* stateSet = material->getOsgStateSet();

	EXPECT_EQ(0u, stateSet->getTextureAttributeList().size());

	material->addUniform(uniform2d0);
	EXPECT_EQ(1u, stateSet->getTextureAttributeList().size());
	EXPECT_FALSE(stateSet->getTextureAttributeList()[0].empty());

	material->addUniform(uniform2d1);
	EXPECT_EQ(9u, stateSet->getTextureAttributeList().size());
	EXPECT_FALSE(stateSet->getTextureAttributeList()[0].empty());
	EXPECT_TRUE(stateSet->getTextureAttributeList()[1].empty());
	EXPECT_FALSE(stateSet->getTextureAttributeList()[8].empty());

	material->addUniform(uniform2d2);
	EXPECT_EQ(9u, stateSet->getTextureAttributeList().size());
	EXPECT_FALSE(stateSet->getTextureAttributeList()[0].empty());
	EXPECT_FALSE(stateSet->getTextureAttributeList()[1].empty());
	EXPECT_TRUE(stateSet->getTextureAttributeList()[2].empty());
	EXPECT_FALSE(stateSet->getTextureAttributeList()[8].empty());
}


/// Expose a bug where a texture uniform could be created that is not really a correctly specialized
/// uniform, if this fails then the uniform2d was not created correctly
TEST(OsgTextureUniformTests, TextureUniformTemplateProblem)
{
	auto material = std::make_shared<OsgMaterial>();
	material->initialize(runtime);
	auto uniform2d = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("TextureUniform");
	auto texture2d = std::make_shared<OsgTexture2d>();
	texture2d->setSize(256, 256);
	uniform2d->set(texture2d);

	material->addUniform(uniform2d);

	osg::StateSet* stateSet = material->getOsgStateSet();

	EXPECT_EQ(1u, stateSet->getTextureAttributeList().size());
}


}; // namespace Graphics
}; // namespace SurgSim