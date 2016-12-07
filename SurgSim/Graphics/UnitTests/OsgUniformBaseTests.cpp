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
/// Tests for the OsgUniformBase class.

#include "SurgSim/Graphics/OsgUniformBase.h"

#include <gtest/gtest.h>

namespace SurgSim
{
namespace Graphics
{

/// OsgUniformBase class for testing
class MockOsgUniformBase : public OsgUniformBase
{
public:
	/// Constructor
	/// \param	name	Name used in shader code to access this uniform
	explicit MockOsgUniformBase(const std::string& name) : OsgUniformBase(name)
	{
	}

	void set(const YAML::Node&)
	{

	}

	virtual const std::string getGlslType() const override
	{
		return "";
	}

};

TEST(OsgUniformBaseTests, InitTest)
{
	MockOsgUniformBase uniform("test name");

	EXPECT_EQ("test name", uniform.getName());

	EXPECT_NE(nullptr, uniform.getOsgUniform());
}

TEST(OsgUniformBaseTests, StateSetTest)
{
	MockOsgUniformBase uniform("test name");

	// Create an OSG state set
	osg::ref_ptr<osg::StateSet> stateSet = new osg::StateSet();

	const osg::StateSet::UniformList& uniforms = stateSet->getUniformList();
	EXPECT_EQ(0u, uniforms.size());

	/// Add the uniform to the state set
	uniform.addToStateSet(stateSet.get());

	EXPECT_EQ(1u, uniforms.size()) << "State set has no uniforms, one should have been added!";
	EXPECT_EQ(uniform.getOsgUniform(), uniforms.at("test name").first) <<
			"First uniform in state set should be the added uniform!";

	/// Remove the uniform from the state set
	uniform.removeFromStateSet(stateSet.get());

	EXPECT_EQ(0u, uniforms.size())
			<< "State set should no longer have any uniforms, the uniform should have been removed!";
}

}  // namespace Graphics
}  // namespace SurgSim