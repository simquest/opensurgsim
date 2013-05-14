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
/// Tests that exercise the functionality of our rigid transform typedefs, which
/// come straight from Eigen.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "gtest/gtest.h"

template <class T>
class RigidTransformTestBase : public testing::Test
{
public:
	typedef T RigidTransform;
};


template <class T>
class RigidTransform3Tests : public RigidTransformTestBase<T>
{
};

typedef ::testing::Types<SurgSim::Math::RigidTransform3d,
						 SurgSim::Math::RigidTransform3f> RigidTransform3Variants;
TYPED_TEST_CASE(RigidTransform3Tests, RigidTransform3Variants);


template <class T>
class AllRigidTransformTests : public RigidTransformTestBase<T>
{
};

typedef ::testing::Types<SurgSim::Math::RigidTransform2d,
						 SurgSim::Math::RigidTransform2f,
						 SurgSim::Math::RigidTransform3d,
						 SurgSim::Math::RigidTransform3f> AllRigidTransformVariants;
TYPED_TEST_CASE(AllRigidTransformTests, AllRigidTransformVariants);


/// Test that rigid transforms can be constructed
TYPED_TEST(AllRigidTransformTests, CanConstruct)
{
	typename TestFixture::RigidTransform transform;
}
