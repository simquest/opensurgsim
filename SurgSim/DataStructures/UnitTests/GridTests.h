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

#ifndef SURGSIM_DATASTRUCTURES_UNITTESTS_GRIDTESTS_H
#define SURGSIM_DATASTRUCTURES_UNITTESTS_GRIDTESTS_H

#include <gtest/gtest.h>

#include "SurgSim/Math/Vector.h"
#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

namespace SurgSim
{
namespace DataStructures
{

template <typename T, size_t N>
class GridTestBase : public testing::Test
{
public:
	/// The test parameters
	typedef T TypeElement;
	static const size_t dimension = N;

	/// The grid type for this test
	typedef MockGrid<TypeElement, dimension> GridType;

	/// Useful vector type of various vectors
	typedef Eigen::Matrix<double, dimension, 1> VectorND;

	/// Useful variables
	VectorND m_size;

	/// Grid min and max n-d vectors
	Eigen::AlignedBox<double, dimension> m_aabb;
	Eigen::AlignedBox<double, dimension> m_aabb1Cell;
	Eigen::AlignedBox<double, dimension> m_aabbBig;

	void SetUp() override
	{
		m_size = VectorND::LinSpaced(0.6, 1.67);

		VectorND numCells = VectorND::LinSpaced(16.45, 257.43);

		m_aabb.min() = -(numCells.cwiseProduct(m_size) * 0.5);
		m_aabb.max() = numCells.cwiseProduct(m_size) * 0.5;

		m_aabb1Cell.min() = -(m_size * 0.5);
		m_aabb1Cell.max() = m_size * 0.5;

		size_t architectureSize = sizeof(size_t) * 8;
		m_aabbBig.min() = -static_cast<double>(static_cast<size_t>(1u) << (architectureSize - 2)) * m_size * 0.5;
		m_aabbBig.max() = static_cast<double>(static_cast<size_t>(1u) << (architectureSize - 2)) * m_size * 0.5;
	}
};

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_UNITTESTS_GRIDTESTS_H
