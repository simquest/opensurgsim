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
/// Tests that exercise the functionality of our quaternion typedefs, which
/// come straight from Eigen.

#include <math.h>
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/MathConvert.h"
#include "gtest/gtest.h"

// Define test fixture class templates.
// We don't really need fixtures as such, but the templatization encodes type.

template <class T>
class AngleAxisTests : public testing::Test
{
public:
	typedef typename T AngleAxis;
	typedef typename T::Scalar Scalar;
};
typedef ::testing::Types<Eigen::AngleAxisd, Eigen::AngleAxisf> AngleAxisVariants;
TYPED_TEST_CASE(AngleAxisTests, AngleAxisVariants);

// Now we're ready to start testing...

// Test conversion to and from yaml node
TYPED_TEST(AngleAxisTests, YamlConvert)
{
	typedef typename TestFixture::AngleAxis AngleAxis;
	typedef typename TestFixture::Scalar T;

	AngleAxis angleAxis;
	angleAxis.angle() = T(0.1);
	angleAxis.axis()[0] = T(1.2);
	angleAxis.axis()[1] = T(2.3);
	angleAxis.axis()[2] = T(3.4);

	YAML::Node node;

	ASSERT_NO_THROW(node = angleAxis);

	EXPECT_TRUE(node.IsSequence());
	EXPECT_EQ(2u, node.size());
	YAML::Node axisNode = node[1];
	EXPECT_TRUE(axisNode.IsSequence());
	EXPECT_EQ(3u, axisNode.size());

	ASSERT_NO_THROW({ AngleAxis expected = node.as<AngleAxis>(); });
	EXPECT_TRUE(angleAxis.isApprox(node.as<AngleAxis>()));
}

