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
/// Tests for conversions to and from OSG quaternion types

#include <SurgSim/Graphics/OsgQuaternionConversions.h>

#include <SurgSim/Math/Vector.h>

#include "gtest/gtest.h"

using SurgSim::Graphics::fromOsg;
using SurgSim::Graphics::toOsg;
using SurgSim::Math::Quaternionf;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector4d;


TEST(OsgQuaternionConversionsTests, QuaternionfTest)
{
	Quaternionf quaternion = Quaternionf(Vector4f::Random());
	osg::Quat osgQuaternion = toOsg(quaternion);
	EXPECT_TRUE(quaternion.isApprox(fromOsg<float>(osgQuaternion)));
}

TEST(OsgQuaternionConversionsTests, QuaterniondTest)
{
	Quaterniond quaternion = Quaterniond(Vector4d::Random());
	osg::Quat osgQuaternion = toOsg(quaternion);
	EXPECT_TRUE(quaternion.isApprox(fromOsg<double>(osgQuaternion)));
}
