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

#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <tuple>

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Grid.h"
#include "SurgSim/Particles/UnitTests/MockObject.h"

using std::tuple;

class ElementTest
{
public:
	int m_number;
	std::string m_string;

	ElementTest() : m_number(-1), m_string("Empty"){}

	explicit ElementTest(int i) : m_number(i) { std::stringstream s; s << i; m_string = s.str(); }

	ElementTest(const ElementTest& e) : m_number(e.m_number), m_string(e.m_string){}

	bool operator ==(const ElementTest& e) const
	{
		return m_number == e.m_number && m_string.compare(e.m_string) == 0;
	}
};
// Add a hash function for this class
namespace std {
	template <>
	struct hash<ElementTest>
	{
		std::size_t operator()(const ElementTest& k) const
		{
			using std::size_t;
			using std::hash;
			using std::string;

			// Compute individual hash values for first,
			// second and third and combine them using XOR
			// and bit shifting:
			return (hash<string>()(k.m_string) ^ (hash<int>()(k.m_number) << 1));
		}
	};
}

namespace SurgSim
{
namespace Particles
{

/// TypeValue template to pack value as template parameters
template <size_t N> class TypeValue
{
public:
	static const size_t value = N;
};
template <size_t N> const size_t TypeValue<N>::value;

/// GridTestBase using a tuple<TypeElement, dimension>
template <typename T>
class GridTestBase : public testing::Test
{
public:
	/// The test parameters
	typedef typename std::tuple_element<0, T>::type TypeElement;
	static const size_t dimension = std::tuple_element<1, T>::type::value;

	/// The grid type for this test
	typedef SurgSim::Particles::MockGrid<TypeElement, dimension> GridType;

	/// Useful vector type of various vectors
	typedef Eigen::Matrix<size_t, dimension, 1> UintVectorND;
	UintVectorND validPowerOf2Dimension1Cell;
	UintVectorND invalidTooBigPowerOf2Dimension;
	UintVectorND validPowerOf2Dimension;
	UintVectorND validNumCellsPerDimension;
	UintVectorND validOffsetPerDimension;
	UintVectorND validOffsetPowerOf2PerDimension;

	/// Useful variables
	double m_size;

	virtual void SetUp() override
	{
		m_size = 0.6;

		validPowerOf2Dimension1Cell = UintVectorND::Constant(0u);
		invalidTooBigPowerOf2Dimension = UintVectorND::Constant(96u);
		validPowerOf2Dimension = UintVectorND::Constant(4u);
		validNumCellsPerDimension = UintVectorND::Constant(16u); // 2^4

		// Offsets are {2^{powerOf2[1]+...+powerOf2[N-1]} , ... 2^{powerOf2[N-1]}, 2^{0}}
		if (dimension >= 1)
		{
			validOffsetPowerOf2PerDimension[dimension - 1] = 0u;
			validOffsetPerDimension[dimension - 1] =
				static_cast<size_t>(pow(2, validOffsetPowerOf2PerDimension[dimension - 1]));
		}
		for (int i = dimension - 2; i >= 0; --i)
		{
			validOffsetPowerOf2PerDimension[i] = validOffsetPowerOf2PerDimension[i + 1] + validPowerOf2Dimension[i + 1];
			validOffsetPerDimension[i] = static_cast<size_t>(pow(2, validOffsetPowerOf2PerDimension[i]));
		}
	}
};

typedef ::testing::Types<
	tuple<size_t, TypeValue<1>>,
	tuple<int, TypeValue<1>>,
	tuple<float, TypeValue<1>>,
	tuple<double, TypeValue<1>>,
	tuple<ElementTest, TypeValue<1>>,
	tuple<size_t, TypeValue<2>>,
	tuple<int, TypeValue<2>>,
	tuple<float, TypeValue<2>>,
	tuple<double, TypeValue<2>>,
	tuple<ElementTest, TypeValue<2>>,
	tuple<size_t, TypeValue<3>>,
	tuple<int, TypeValue<3>>,
	tuple<float, TypeValue<3>>,
	tuple<double, TypeValue<3>>,
	tuple<ElementTest, TypeValue<3>>> MyTypes;
TYPED_TEST_CASE(GridTestBase, MyTypes);

TEST(Grid, PowerOf3StaticAssertionTest)
{
	static_assert(powerOf3<0>::value == 1, "3^0 != 1");
	static_assert(powerOf3<1>::value == 3, "3^1 != 3");
	static_assert(powerOf3<2>::value == 9, "3^2 != 9");
	static_assert(powerOf3<3>::value == 27, "3^3 != 27");
}

TYPED_TEST(GridTestBase, ConstructorTest)
{
	typedef SurgSim::Particles::Grid<TypeElement, dimension> GridType;

	ASSERT_NO_THROW({GridType grid;});
}

TYPED_TEST(GridTestBase, InitTest)
{
	ASSERT_NO_THROW({GridType grid; grid.init(m_size, validPowerOf2Dimension1Cell);});
	ASSERT_NO_THROW({GridType grid; grid.init(m_size, validPowerOf2Dimension);});
	ASSERT_THROW({GridType grid; grid.init(m_size, invalidTooBigPowerOf2Dimension);},
		SurgSim::Framework::AssertionFailure);

	GridType grid;
	grid.init(m_size, validPowerOf2Dimension);
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getMappingElementToCellId().size());
	ASSERT_DOUBLE_EQ(m_size, grid.getSize());
	ASSERT_TRUE(grid.getNumCellsPerDimension() == validNumCellsPerDimension);
	ASSERT_TRUE(grid.getPowerOf2CellsPerDimension() == validPowerOf2Dimension);
	ASSERT_TRUE(grid.getOffsetPerDimension() == validOffsetPerDimension);
	ASSERT_TRUE(grid.getOffsetPowerOf2PerDimension() == validOffsetPowerOf2PerDimension);
	ASSERT_EQ(validNumCellsPerDimension.prod(), grid.getNumCells());
	ASSERT_TRUE(grid.getAABB().min().isApprox(-validNumCellsPerDimension.cast<double>() * m_size * 0.5));
	ASSERT_TRUE(grid.getAABB().max().isApprox(validNumCellsPerDimension.cast<double>() * m_size * 0.5));
}

TYPED_TEST(GridTestBase, AddElementAtTest)
{
	GridType grid;
	grid.init(m_size, validPowerOf2Dimension);

	// Add an element outside of the grid
	grid.addElementAt(validNumCellsPerDimension.cast<double>() * 3.0 * m_size, TypeElement());
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getMappingElementToCellId().size());
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element outside of the grid
	grid.addElementAt(-validNumCellsPerDimension.cast<double>() * 3.0 * m_size, TypeElement());
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getMappingElementToCellId().size());
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid
	TypeElement e0(0), e1(1);
	grid.addElementAt(Eigen::Matrix<double, dimension, 1>::Zero(), e0);
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(1u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_ANY_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid, in the same cell
	grid.addElementAt(Eigen::Matrix<double, dimension, 1>::Zero(), e1);
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(2u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_EQ(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e1]);
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid, in a different cell
	TypeElement e2(2);
	grid.addElementAt(Eigen::Matrix<double, dimension, 1>::Ones() * m_size * 1.5, e2);
	ASSERT_EQ(2u, grid.getActiveCells().size());
	ASSERT_EQ(3u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e2));
	ASSERT_EQ(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e1]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e2]);
	ASSERT_NE(grid.getMappingElementToCellId()[e1], grid.getMappingElementToCellId()[e2]);
	ASSERT_EQ(0u, grid.getNeighborsMap().size());
}

TYPED_TEST(GridTestBase, NeighborsTest)
{
	GridType grid;
	grid.init(m_size, validPowerOf2Dimension);

	// Build the following grid:
	// Cell(e0, e1) <- neighbor -> Cell(e2) <- neighbor -> Cell(e3) <- ..not neighbor.. -> Cell(e4)

	Eigen::Matrix<double, dimension, 1> position = Eigen::Matrix<double, dimension, 1>::Zero();

	// Add an element inside of the grid
	TypeElement e0(0), e1(1);
	grid.addElementAt(position, e0);
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(1u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_ANY_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid, in the same cell
	grid.addElementAt(position, e1);
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(2u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_EQ(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e1]);
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid, in a different cell
	position[0] += m_size * 1.1; // Next cell on the 1st dimension
	TypeElement e2(2);
	grid.addElementAt(position, e2);
	ASSERT_EQ(2u, grid.getActiveCells().size());
	ASSERT_EQ(3u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e2));
	ASSERT_EQ(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e1]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e2]);
	ASSERT_NE(grid.getMappingElementToCellId()[e1], grid.getMappingElementToCellId()[e2]);
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid, in a different cell
	position[0] += m_size * 1.1; // Next cell on the 1st dimension
	TypeElement e3(3);
	grid.addElementAt(position, e3);
	ASSERT_EQ(3u, grid.getActiveCells().size());
	ASSERT_EQ(4u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e2));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e3));
	ASSERT_EQ(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e1]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e2]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e3]);
	ASSERT_NE(grid.getMappingElementToCellId()[e2], grid.getMappingElementToCellId()[e3]);
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	// Add an element inside of the grid, far away from all other elements
	position[0] += m_size * 2.1; // Few cells further on the 1st dimension
	TypeElement e4(4);
	grid.addElementAt(position, e4);
	ASSERT_EQ(4u, grid.getActiveCells().size());
	ASSERT_EQ(5u, grid.getMappingElementToCellId().size());
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e0));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e1));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e2));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e3));
	ASSERT_NO_THROW(grid.getMappingElementToCellId().at(e4));
	ASSERT_EQ(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e1]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e2]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e3]);
	ASSERT_NE(grid.getMappingElementToCellId()[e2], grid.getMappingElementToCellId()[e3]);
	ASSERT_NE(grid.getMappingElementToCellId()[e0], grid.getMappingElementToCellId()[e4]);
	ASSERT_NE(grid.getMappingElementToCellId()[e1], grid.getMappingElementToCellId()[e4]);
	ASSERT_NE(grid.getMappingElementToCellId()[e2], grid.getMappingElementToCellId()[e4]);
	ASSERT_NE(grid.getMappingElementToCellId()[e3], grid.getMappingElementToCellId()[e4]);
	ASSERT_EQ(0u, grid.getNeighborsMap().size());

	grid.computeNeighborsMap();
	ASSERT_EQ(5u, grid.getNeighborsMap().size());
	ASSERT_NO_THROW(grid.getNeighborsMap().at(e0));
	ASSERT_NO_THROW(grid.getNeighborsMap().at(e1));
	ASSERT_NO_THROW(grid.getNeighborsMap().at(e2));
	ASSERT_NO_THROW(grid.getNeighborsMap().at(e3));
	ASSERT_NO_THROW(grid.getNeighborsMap().at(e4));

	// e0's neighbors = {e0, e1, e2}
	auto& e0Neighbors = grid.getNonConstNeighborsMap()[e0];
	ASSERT_EQ(3u, e0Neighbors.size());
	ASSERT_TRUE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e0) != e0Neighbors.end());
	ASSERT_TRUE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e1) != e0Neighbors.end());
	ASSERT_TRUE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e2) != e0Neighbors.end());
	ASSERT_FALSE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e3) != e0Neighbors.end());
	ASSERT_FALSE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e4) != e0Neighbors.end());

	// e1's neighbors = {e0, e1, e2}
	auto& e1Neighbors = grid.getNonConstNeighborsMap()[e1];
	ASSERT_EQ(3u, e1Neighbors.size());
	ASSERT_TRUE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e0) != e1Neighbors.end());
	ASSERT_TRUE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e1) != e1Neighbors.end());
	ASSERT_TRUE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e2) != e1Neighbors.end());
	ASSERT_FALSE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e3) != e1Neighbors.end());
	ASSERT_FALSE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e4) != e1Neighbors.end());

	// e2's neighbors = {e0, e1, e2, e3}
	auto& e2Neighbors = grid.getNonConstNeighborsMap()[e2];
	ASSERT_EQ(4u, e2Neighbors.size());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e0) != e2Neighbors.end());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e1) != e2Neighbors.end());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e2) != e2Neighbors.end());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e3) != e2Neighbors.end());
	ASSERT_FALSE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e4) != e2Neighbors.end());

	// e3's neighbors = {e2, e3}
	auto& e3Neighbors = grid.getNonConstNeighborsMap()[e3];
	ASSERT_EQ(2u, e3Neighbors.size());
	ASSERT_FALSE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e0) != e3Neighbors.end());
	ASSERT_FALSE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e1) != e3Neighbors.end());
	ASSERT_TRUE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e2) != e3Neighbors.end());
	ASSERT_TRUE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e3) != e3Neighbors.end());
	ASSERT_FALSE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e4) != e3Neighbors.end());

	// e4's neighbors = {e4}
	auto& e4Neighbors = grid.getNonConstNeighborsMap()[e4];
	ASSERT_EQ(1u, e4Neighbors.size());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e0) != e4Neighbors.end());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e1) != e4Neighbors.end());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e2) != e4Neighbors.end());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e3) != e4Neighbors.end());
	ASSERT_TRUE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e4) != e4Neighbors.end());
}

TYPED_TEST(GridTestBase, ResetTest)
{
	GridType grid;
	grid.init(m_size, validPowerOf2Dimension);

	TypeElement e0(0), e1(1);
	grid.addElementAt(Eigen::Matrix<double, dimension, 1>::Zero(), e0);
	grid.addElementAt(Eigen::Matrix<double, dimension, 1>::Zero(), e1);
	grid.computeNeighborsMap();

	ASSERT_NO_THROW(grid.reset());
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getMappingElementToCellId().size());
	ASSERT_EQ(0u, grid.getNeighborsMap().size());
}

}; // namespace Particles
}; // namespace SurgSim
