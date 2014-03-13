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

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <string>
#include <tuple>

#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"

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
											   double abs_err)
{
	if ((expected.rows() != actual.rows()) || (expected.cols() != actual.cols()))
	{
		return ::testing::AssertionFailure()
			<< "Sizes do not match." << std::endl
			<< "Actual: [" << actual.rows() << ", " << actual.cols() << "]" << std::endl
			<< "Expected: [" << expected.rows() << ", " << expected.cols() << "]" << std::endl;
	}

	if (!expected.isApprox(actual, abs_err))
	{
		::testing::AssertionResult result = ::testing::AssertionFailure()
			<< "Matrices are not within epsilon of each other." << std::endl
			<< "Difference: " << (actual - expected).norm() << std::endl
			<< "Epsilon: " << abs_err << std::endl;

		if (expected.cols() == 1 || expected.rows() == 1)
		{
			typedef std::tuple<int, double, double> DifferenceTuple;
			std::vector<DifferenceTuple> differences;
			for (int index = 0; index < expected.size(); ++index)
			{
				if (std::abs(expected(index) - actual(index)) > abs_err)
				{
					differences.emplace_back(index, expected(index), actual(index));
				}
			}

			result << "Total differing values: " << differences.size() << std::endl;

			for (std::vector<DifferenceTuple>::iterator it = std::begin(differences); it != std::end(differences); ++it)
			{
				result
					<< "[Index, Expected, Actual]: "
					<< std::get<0>(*it) << ", " << std::get<1>(*it)  << ", " << std::get<2>(*it)  << std::endl;
			}
		}

		return result;
	}

	return ::testing::AssertionSuccess();
}

} // namespace Testing
} // namespace Physics
} // namespace SurgSim
