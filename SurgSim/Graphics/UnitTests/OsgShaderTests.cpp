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
/// Tests for the OsgShader class.

#include <SurgSim/Graphics/OsgShader.h>

#include <gtest/gtest.h>

namespace SurgSim
{
namespace Graphics
{

TEST(OsgShaderTests, InitTest)
{
	OsgShader shader;

	EXPECT_NE(nullptr, shader.getOsgProgram());
}

TEST(OsgShaderTests, StateSetTest)
{
	OsgShader shader;

	// Create an OSG state set
	osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet();

	const osg::StateSet::AttributeList& attributes = stateSet->getAttributeList();
	EXPECT_EQ(0u, attributes.size());

	/// Add the shader to the state set
	shader.addToStateSet(stateSet.get());

	EXPECT_EQ(1u, attributes.size()) << "State set has no attributes, one shader program should have been added!";
	EXPECT_EQ(shader.getOsgProgram(), attributes.at(osg::StateAttribute::TypeMemberPair(
		osg::StateAttribute::PROGRAM, 0)).first) << "First attribute in state set should be the added shader program!";

	/// Remove the shader from the state set
	shader.removeFromStateSet(stateSet.get());

	EXPECT_EQ(0u, attributes.size()) << 
		"State set should no longer have any attributes, the shader program should have been removed!";
}

}  // namespace Graphics
}  // namespace SurgSim