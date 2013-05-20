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
/// Tests for conversions to and from OSG rigid transform types

#include <SurgSim/Graphics/OsgQuaternionConversions.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>
#include <SurgSim/Graphics/OsgVectorConversions.h>

#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

#include "gtest/gtest.h"

using SurgSim::Graphics::fromOsg;
using SurgSim::Graphics::toOsg;
using SurgSim::Math::Quaternionf;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform2f;
using SurgSim::Math::RigidTransform2d;
using SurgSim::Math::RigidTransform3f;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3f;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Vector4d;
using SurgSim::Math::makeRigidTransform;


TEST(OsgRigidTransformConversionsTests, RigidTransform3fTest)
{
	Quaternionf rotation = Quaternionf(Vector4f::Random());
	rotation.normalize();
	Vector3f translation = Vector3f::Random();
	RigidTransform3f transform = makeRigidTransform(rotation, translation);

	std::pair<osg::Quat, osg::Vec3f> osgTransform = toOsg(transform);

	EXPECT_TRUE(rotation.isApprox(fromOsg<float>(osgTransform.first)));
	EXPECT_TRUE(translation.isApprox(fromOsg(osgTransform.second)));
}

TEST(OsgRigidTransformConversionsTests, RigidTransform3fMultiplyTest)
{
	Quaternionf rotation = Quaternionf(Vector4f::Random());
	rotation.normalize();
	Vector3f translation = Vector3f::Random();
	RigidTransform3f transform = makeRigidTransform(rotation, translation);

	Vector3f vector = Vector3f::Random();
	osg::Vec3f osgVector = toOsg(vector);

	Vector3f result = transform * vector;

	std::pair<osg::Quat, osg::Vec3f> osgTransform = toOsg(transform);

	osg::Vec3f osgResult = osgTransform.first * osgVector + osgTransform.second;

	EXPECT_TRUE(result.isApprox(fromOsg(osgResult)));
}

TEST(OsgRigidTransformConversionsTests, RigidTransform3dTest)
{
	Quaterniond rotation = Quaterniond(Vector4d::Random());
	rotation.normalize();
	Vector3d translation = Vector3d::Random();
	RigidTransform3d transform = makeRigidTransform(rotation, translation);

	std::pair<osg::Quat, osg::Vec3d> osgTransform = toOsg(transform);

	EXPECT_TRUE(rotation.isApprox(fromOsg<double>(osgTransform.first)));
	EXPECT_TRUE(translation.isApprox(fromOsg(osgTransform.second)));
}

TEST(OsgRigidTransformConversionsTests, RigidTransform3dMultiplyTest)
{
	Quaterniond rotation = Quaterniond(Vector4d::Random());
	rotation.normalize();
	Vector3d translation = Vector3d::Random();
	RigidTransform3d transform = makeRigidTransform(rotation, translation);

	Vector3d vector = Vector3d::Random();
	osg::Vec3d osgVector = toOsg(vector);

	Vector3d result = transform * vector;

	std::pair<osg::Quat, osg::Vec3d> osgTransform = toOsg(transform);

	osg::Vec3d osgResult = osgTransform.first * osgVector + osgTransform.second;

	EXPECT_TRUE(result.isApprox(fromOsg(osgResult)));
}
