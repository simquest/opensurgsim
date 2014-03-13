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

#ifndef SURGSIM_PHYSICS_UNITTESTS_EIGENGTESTASSERTS_H
#define SURGSIM_PHYSICS_UNITTESTS_EIGENGTESTASSERTS_H

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace SurgSim
{
namespace Physics
{
namespace Testing
{
::testing::AssertionResult AssertMatricesEqual(const char *expected_expr,
											   const char *actual_expr,
											   const char *abs_err_expr,
											   const Eigen::MatrixXd &expected,
											   const Eigen::MatrixXd &actual,
											   double abs_err);

} // namespace Testing
} // namespace Physics
} // namespace SurgSim

#define EXPECT_NEAR_EIGEN(expected, actual, abs_err) \
	EXPECT_PRED_FORMAT3(SurgSim::Physics::Testing::AssertMatricesEqual, (expected), (actual), (abs_err))
#define ASSERT_NEAR_EIGEN(expected, actual, abs_err) \
	ASSERT_PRED_FORMAT3(SurgSim::Physics::Testing::AssertMatricesEqual, (expected), (actual), (abs_err))

#endif // SURGSIM_PHYSICS_UNITTESTS_EIGENGTESTASSERTS_H
