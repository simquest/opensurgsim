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
/// Tests that exercise the functionality of our vector typedefs, which come
/// straight from Eigen.

#include <math.h>
#include "SurgSim/Math/Load.h"
#include "gtest/gtest.h"

// Define test fixture class templates.
// We don't really need fixtures as such, but the templatization encodes type.

template <class T>
class LoadTests : public testing::Test
{
public:
	typedef typename T::Scalar Scalar;
	typedef T Load;
};


typedef ::testing::Types<SurgSim::Math::Loadd,
						 SurgSim::Math::Loadf> LoadVariants;
TYPED_TEST_CASE(LoadTests, LoadVariants);


// ==================== CONSTRUCTION & INITIALIZATION ====================

/// Test that loads can be constructed.
TYPED_TEST(LoadTests, CanConstruct)
{
	typedef typename TestFixture::Load Load;
	//typedef typename TestFixture::Scalar T;

	Load instance;
	Load another(instance);
}

TYPED_TEST(LoadTests, DefaultConstructorInitialization)
{
	typedef typename TestFixture::Load Load;
	typedef typename Eigen::Matrix<TestFixture::Scalar, 3, 1, Eigen::DontAlign> Vector3;

	Load load;
	EXPECT_NEAR(0.0, load.getForce().squaredNorm(), 1e-12) <<
		"initialization was incorrect: force = " << load.getForce();
	EXPECT_NEAR(0.0, load.getMomentAroundPoint(Vector3::Zero()).squaredNorm(), 1e-12) <<
		"initialization was incorrect: moment around O = " <<
		load.getMomentAroundPoint(Vector3::Zero());
	EXPECT_NEAR(0.0, load.getMomentAroundPoint(Vector3::Constant(1)).squaredNorm(), 1e-12) <<
		"initialization was incorrect: moment around (1,1,1) = " <<
		load.getMomentAroundPoint(Vector3::Constant(1));
}

TYPED_TEST(LoadTests, ZeroValue)
{
	typedef typename TestFixture::Load Load;
	typedef typename Eigen::Matrix<TestFixture::Scalar, 3, 1, Eigen::DontAlign> Vector3;

	EXPECT_NEAR(0.0, Load::Zero().getForce().squaredNorm(), 1e-12) <<
		"Zero() value was incorrect: force = " << Load::Zero().getForce();
	EXPECT_NEAR(0.0, Load::Zero().getMomentAroundPoint(Vector3::Zero()).squaredNorm(), 1e-12) <<
		"Zero() value was incorrect: moment around O = " <<
		Load::Zero().getMomentAroundPoint(Vector3::Zero());
	EXPECT_NEAR(0.0, Load::Zero().getMomentAroundPoint(Vector3::Constant(1)).squaredNorm(), 1e-12) <<
		"Zero() value was incorrect: moment around (1,1,1) = " <<
		Load::Zero().getMomentAroundPoint(Vector3::Constant(1));
}

TYPED_TEST(LoadTests, SetToZero)
{
	typedef typename TestFixture::Load Load;
	typedef typename Eigen::Matrix<TestFixture::Scalar, 3, 1, Eigen::DontAlign> Vector3;

	Load load;
	load.addForceAtPoint(Vector3::UnitY(), Vector3::UnitX());
	load.setZero();
	EXPECT_NEAR(0.0, load.getForce().squaredNorm(), 1e-12) <<
		"setZero() was incorrect: force = " << load.getForce();
	EXPECT_NEAR(0.0, load.getMomentAroundPoint(Vector3::Zero()).squaredNorm(), 1e-12) <<
		"setZero() was incorrect: moment around O = " <<
		load.getMomentAroundPoint(Vector3::Zero());
	EXPECT_NEAR(0.0, load.getMomentAroundPoint(Vector3::Constant(1)).squaredNorm(), 1e-12) <<
		"setZero() was incorrect: moment around (1,1,1) = " <<
		load.getMomentAroundPoint(Vector3::Constant(1));
}

// ==================== COMPUTING LOADS ====================


// ==================== ARITHMETIC ====================

TYPED_TEST(LoadTests, Assign)
{
}

TYPED_TEST(LoadTests, Negate)
{
}

TYPED_TEST(LoadTests, Add)
{
}

TYPED_TEST(LoadTests, Subtract)
{
}

TYPED_TEST(LoadTests, AddTo)
{
}

TYPED_TEST(LoadTests, SubtractFrom)
{
}

// ==================== TYPE CONVERSION ====================

TYPED_TEST(LoadTests, TypeCasting)
{
	typedef typename TestFixture::Load Load;

	Load load;
	// XXX TODO INITIALIZE TO NON-ZERO XXX

	// Ugh, "template" is required to get this to parse properly.  This is
	// triggered because the test is a part of a template class; you don't
	// need to do this in a non-template context.
	SurgSim::Math::Loadd loadd = load.template cast<double>();
	//XXX EXPECT_NEAR(expectedSum, vd.sum(), 1e-6);
	SurgSim::Math::Loadf loadf = load.template cast<float>();
	//XXX EXPECT_NEAR(expectedSum, vf.sum(), 1e-4);
}

// ==================== MISCELLANEOUS ====================

