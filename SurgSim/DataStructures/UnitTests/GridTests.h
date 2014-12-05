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

#ifndef SURGSIM_DATASTRUCTURES_GRIDTESTS_H
#define SURGSIM_DATASTRUCTURES_GRIDTESTS_H

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
	typedef Eigen::Matrix<size_t, dimension, 1> UintVectorND;
	typedef Eigen::Matrix<double, dimension, 1> VectorND;
	UintVectorND validPowerOf2Dimension;
	UintVectorND validNumCellsPerDimension;
	UintVectorND validOffsetPerDimension;
	UintVectorND validOffsetPowerOf2PerDimension;

	/// Useful variables
	VectorND m_size;

	/// Grid min and max n-d vectors
	Eigen::AlignedBox<double, dimension> m_aabb;
	Eigen::AlignedBox<double, dimension> m_aabb1Cell;
	Eigen::AlignedBox<double, dimension> m_aabbTooBig;

	virtual void SetUp() override
	{
		m_size = VectorND::LinSpaced(0.6, 1.67);

		validPowerOf2Dimension = UintVectorND::Constant(4u);
		validNumCellsPerDimension = UintVectorND::Constant(16u); // 2^4

		m_aabb.min() = -(validNumCellsPerDimension.template cast<double>().cwiseProduct(m_size) * 0.5);
		m_aabb.max() = validNumCellsPerDimension.template cast<double>().cwiseProduct(m_size) * 0.5;

		m_aabb1Cell.min() = -(m_size * 0.5);
		m_aabb1Cell.max() = m_size * 0.5;

		size_t architectureSize = sizeof(size_t) * 8;
		m_aabbTooBig.min() = -static_cast<double>(static_cast<size_t>(1u) << (architectureSize - 2)) * m_size * 0.5;
		m_aabbTooBig.max() = static_cast<double>(static_cast<size_t>(1u) << (architectureSize - 2)) * m_size * 0.5;

		// Offsets are {2^{powerOf2[1]+...+powerOf2[N-1]} , ... 2^{powerOf2[N-1]}, 2^{0}}
		if (dimension >= 1)
		{
			validOffsetPowerOf2PerDimension[dimension - 1] = 0u;
			validOffsetPerDimension[dimension - 1] =
				static_cast<size_t>(pow(2, validOffsetPowerOf2PerDimension[dimension - 1]));
		}
		for (int i = static_cast<int>(dimension) - 2; i >= 0; --i)
		{
			validOffsetPowerOf2PerDimension[i] = validOffsetPowerOf2PerDimension[i + 1] + validPowerOf2Dimension[i + 1];
			validOffsetPerDimension[i] = static_cast<size_t>(pow(2, validOffsetPowerOf2PerDimension[i]));
		}
	}
};

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_GRIDTESTS_H
