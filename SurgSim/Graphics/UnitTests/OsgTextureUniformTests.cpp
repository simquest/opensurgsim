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

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgUniform.h>
#include <SurgSim/Graphics/OsgTexture1d.h>
#include <SurgSim/Graphics/OsgTexture2d.h>
#include <SurgSim/Graphics/OsgTexture3d.h>
#include <SurgSim/Graphics/OsgTextureRectangle.h>

#include <osg/StateSet>

namespace SurgSim 
{
namespace Graphics
{


/// Expose a bug where a texture uniform could be created that is not really a correctly specialized
/// uniform, if this fails then the uniform2d was not created correctly
TEST(OsgTextureUniformTests, TextureUniformTemplateProblem)
{
	auto material = std::make_shared<OsgMaterial>();
	auto uniform2d = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture1d>>>("Texture2d");

	material->addUniform(uniform2d);

	osg::StateSet* stateSet = material->getOsgStateSet();

	EXPECT_EQ(1u, stateSet->getTextureAttributeList().size());
}

}; // namespace Graphics
}; // namespace SurgSim