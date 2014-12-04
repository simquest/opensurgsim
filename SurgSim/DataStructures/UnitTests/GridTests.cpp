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
#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

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
namespace DataStructures
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
	typedef MockGrid<TypeElement, dimension> GridType;

	/// Useful vector type of various vectors
	typedef Eigen::Matrix<size_t, dimension, 1> UintVectorND;
	typedef Eigen::Matrix<double, dimension, 1> VectorND;
	UintVectorND validPowerOf2Dimension1Cell;
	UintVectorND invalidTooBigPowerOf2Dimension;
	UintVectorND validPowerOf2Dimension;
	UintVectorND validNumCellsPerDimension;
	UintVectorND validOffsetPerDimension;
	UintVectorND validOffsetPowerOf2PerDimension;

	/// Useful variables
	VectorND m_size;

	/// Grid min and max n-d vectors
	Eigen::Matrix<double, dimension, 1> minExpected, maxExpected;

	virtual void SetUp() override
	{
		m_size = VectorND::LinSpaced(0.6, 1.67);

		validPowerOf2Dimension1Cell = UintVectorND::Constant(0u);
		invalidTooBigPowerOf2Dimension = UintVectorND::Constant(96u);
		validPowerOf2Dimension = UintVectorND::Constant(4u);
		validNumCellsPerDimension = UintVectorND::Constant(16u); // 2^4

		minExpected = -(validNumCellsPerDimension.template cast<double>().cwiseProduct(m_size) * 0.5);
		maxExpected = validNumCellsPerDimension.template cast<double>().cwiseProduct(m_size) * 0.5;

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

template <typename T>
class Grid3DTestBase : public GridTestBase<T>
{
};

typedef ::testing::Types<
	tuple<size_t, TypeValue<3>>,
	tuple<int, TypeValue<3>>,
	tuple<float, TypeValue<3>>,
	tuple<double, TypeValue<3>>,
	tuple<ElementTest, TypeValue<3>>> My3DTypes;
TYPED_TEST_CASE(Grid3DTestBase, My3DTypes);

TEST(Grid, PowerOf3StaticAssertionTest)
{
	static_assert(powerOf3<0>::value == 1, "3^0 != 1");
	static_assert(powerOf3<1>::value == 3, "3^1 != 3");
	static_assert(powerOf3<2>::value == 9, "3^2 != 9");
	static_assert(powerOf3<3>::value == 27, "3^3 != 27");
}

TYPED_TEST(GridTestBase, ConstructorTest)
{
	typedef typename TestFixture::GridType GridType;

	ASSERT_NO_THROW({GridType grid(this->m_size, this->validPowerOf2Dimension1Cell);});
	ASSERT_NO_THROW({GridType grid(this->m_size, this->validPowerOf2Dimension);});
	ASSERT_THROW({GridType grid(this->m_size, this->invalidTooBigPowerOf2Dimension);},
		SurgSim::Framework::AssertionFailure);

	GridType grid(this->m_size, this->validPowerOf2Dimension);
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());
	ASSERT_TRUE(grid.getSize().isApprox(this->m_size));
	ASSERT_TRUE(grid.getNumCells() == this->validNumCellsPerDimension);
	ASSERT_TRUE(grid.getExponents() == this->validPowerOf2Dimension);
	ASSERT_TRUE(grid.getOffsets() == this->validOffsetPerDimension);
	ASSERT_TRUE(grid.getOffsetExponents() == this->validOffsetPowerOf2PerDimension);
	ASSERT_TRUE(grid.getAABB().min().isApprox(this->minExpected));
	ASSERT_TRUE(grid.getAABB().max().isApprox(this->maxExpected));
}

TYPED_TEST(GridTestBase, addElementTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->validPowerOf2Dimension);

	// Add an element outside of the grid
	auto position =  3.0 * this->m_size.cwiseProduct(this->validNumCellsPerDimension.template cast<double>());
	grid.addElement(TypeElement(), position);
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());

	// Add an element outside of the grid
	grid.addElement(TypeElement(), -position);
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());

	// Add an element inside of the grid
	TypeElement e0(0), e1(1);
	grid.addElement(e0, Eigen::Matrix<double, TestFixture::dimension, 1>::Zero());
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(1u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_ANY_THROW(grid.getCellIds().at(e1));

	// Add an element inside of the grid, in the same cell
	grid.addElement(e1, Eigen::Matrix<double, TestFixture::dimension, 1>::Zero());
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(2u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);

	// Add an element inside of the grid, in a different cell
	TypeElement e2(2);
	grid.addElement(e2, this->m_size * 1.5);
	ASSERT_EQ(2u, grid.getActiveCells().size());
	ASSERT_EQ(3u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_NO_THROW(grid.getCellIds().at(e2));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e2]);
	ASSERT_NE(grid.getCellIds()[e1], grid.getCellIds()[e2]);
}

TYPED_TEST(GridTestBase, NeighborsTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->validPowerOf2Dimension);

	// Build the following grid:
	// Cell(e0, e1) <- neighbor -> Cell(e2) <- neighbor -> Cell(e3) <- ..not neighbor.. -> Cell(e4)

	Eigen::Matrix<double, TestFixture::dimension, 1> position =
		Eigen::Matrix<double, TestFixture::dimension, 1>::Zero();

	// Add an element inside of the grid
	TypeElement e0(0), e1(1);
	grid.addElement(e0, position);
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(1u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_ANY_THROW(grid.getCellIds().at(e1));

	// Add an element inside of the grid, in the same cell
	grid.addElement(e1, position);
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(2u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);

	// Add an element inside of the grid, in a different cell
	position[0] += this->m_size[0] * 1.1; // Next cell on the 1st dimension
	TypeElement e2(2);
	grid.addElement(e2, position);
	ASSERT_EQ(2u, grid.getActiveCells().size());
	ASSERT_EQ(3u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_NO_THROW(grid.getCellIds().at(e2));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e2]);
	ASSERT_NE(grid.getCellIds()[e1], grid.getCellIds()[e2]);

	// Add an element inside of the grid, in a different cell
	position[0] += this->m_size[0] * 1.1; // Next cell on the 1st dimension
	TypeElement e3(3);
	grid.addElement(e3, position);
	ASSERT_EQ(3u, grid.getActiveCells().size());
	ASSERT_EQ(4u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_NO_THROW(grid.getCellIds().at(e2));
	ASSERT_NO_THROW(grid.getCellIds().at(e3));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e2]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e3]);
	ASSERT_NE(grid.getCellIds()[e2], grid.getCellIds()[e3]);

	// Add an element inside of the grid, far away from all other elements
	position[0] += this->m_size[0] * 2.1; // Few cells further on the 1st dimension
	TypeElement e4(4);
	grid.addElement(e4, position);
	ASSERT_EQ(4u, grid.getActiveCells().size());
	ASSERT_EQ(5u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_NO_THROW(grid.getCellIds().at(e2));
	ASSERT_NO_THROW(grid.getCellIds().at(e3));
	ASSERT_NO_THROW(grid.getCellIds().at(e4));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e2]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e3]);
	ASSERT_NE(grid.getCellIds()[e2], grid.getCellIds()[e3]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e4]);
	ASSERT_NE(grid.getCellIds()[e1], grid.getCellIds()[e4]);
	ASSERT_NE(grid.getCellIds()[e2], grid.getCellIds()[e4]);
	ASSERT_NE(grid.getCellIds()[e3], grid.getCellIds()[e4]);

	ASSERT_NO_THROW(grid.getNeighbors(e0));
	ASSERT_NO_THROW(grid.getNeighbors(e1));
	ASSERT_NO_THROW(grid.getNeighbors(e2));
	ASSERT_NO_THROW(grid.getNeighbors(e3));
	ASSERT_NO_THROW(grid.getNeighbors(e4));

	// e0's neighbors = {e0, e1, e2}
	auto& e0Neighbors = grid.getNonConstNeighbors(e0);
	ASSERT_EQ(3u, e0Neighbors.size());
	ASSERT_TRUE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e0) != e0Neighbors.end());
	ASSERT_TRUE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e1) != e0Neighbors.end());
	ASSERT_TRUE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e2) != e0Neighbors.end());
	ASSERT_FALSE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e3) != e0Neighbors.end());
	ASSERT_FALSE(std::find(e0Neighbors.begin(), e0Neighbors.end(), e4) != e0Neighbors.end());

	// e1's neighbors = {e0, e1, e2}
	auto& e1Neighbors = grid.getNonConstNeighbors(e1);
	ASSERT_EQ(3u, e1Neighbors.size());
	ASSERT_TRUE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e0) != e1Neighbors.end());
	ASSERT_TRUE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e1) != e1Neighbors.end());
	ASSERT_TRUE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e2) != e1Neighbors.end());
	ASSERT_FALSE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e3) != e1Neighbors.end());
	ASSERT_FALSE(std::find(e1Neighbors.begin(), e1Neighbors.end(), e4) != e1Neighbors.end());

	// e2's neighbors = {e0, e1, e2, e3}
	auto& e2Neighbors = grid.getNonConstNeighbors(e2);
	ASSERT_EQ(4u, e2Neighbors.size());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e0) != e2Neighbors.end());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e1) != e2Neighbors.end());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e2) != e2Neighbors.end());
	ASSERT_TRUE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e3) != e2Neighbors.end());
	ASSERT_FALSE(std::find(e2Neighbors.begin(), e2Neighbors.end(), e4) != e2Neighbors.end());

	// e3's neighbors = {e2, e3}
	auto& e3Neighbors = grid.getNonConstNeighbors(e3);
	ASSERT_EQ(2u, e3Neighbors.size());
	ASSERT_FALSE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e0) != e3Neighbors.end());
	ASSERT_FALSE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e1) != e3Neighbors.end());
	ASSERT_TRUE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e2) != e3Neighbors.end());
	ASSERT_TRUE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e3) != e3Neighbors.end());
	ASSERT_FALSE(std::find(e3Neighbors.begin(), e3Neighbors.end(), e4) != e3Neighbors.end());

	// e4's neighbors = {e4}
	auto& e4Neighbors = grid.getNonConstNeighbors(e4);
	ASSERT_EQ(1u, e4Neighbors.size());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e0) != e4Neighbors.end());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e1) != e4Neighbors.end());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e2) != e4Neighbors.end());
	ASSERT_FALSE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e3) != e4Neighbors.end());
	ASSERT_TRUE(std::find(e4Neighbors.begin(), e4Neighbors.end(), e4) != e4Neighbors.end());

	// Test element not in the grid has no neighbors
	TypeElement e5(5);
	ASSERT_EQ(0u, grid.getNeighbors(e5).size());
}

TYPED_TEST(Grid3DTestBase, Neighbors3DTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->validPowerOf2Dimension);

	// Build a grid where we have 1 element per cell and 1 cell has all its neighbors populated
	// The grid content would be for dimension x=0 (similar on dimension x=1 and x=2):
	// Cell(e000) Cell(e001) Cell(e002)
	// Cell(e010) Cell(e011) Cell(e012)
	// Cell(e020) Cell(e021) Cell(e022)

	// Add the 27 elements inside the grid on 27 different cells forming a cube
	Eigen::Matrix<double, 3, 1> position = Eigen::Matrix<double, 3, 1>::Zero();
	Number<size_t, 3, 3> number; // 3 digits in base 3 => covers 27 different numbers
	const TypeElement element[27] = {
		TypeElement(0), TypeElement(1), TypeElement(2),
		TypeElement(3), TypeElement(4), TypeElement(5),
		TypeElement(6), TypeElement(7), TypeElement(8),
		TypeElement(9), TypeElement(10), TypeElement(11),
		TypeElement(12), TypeElement(13), TypeElement(14),
		TypeElement(15), TypeElement(16), TypeElement(17),
		TypeElement(18), TypeElement(19), TypeElement(20),
		TypeElement(21), TypeElement(22), TypeElement(23),
		TypeElement(24), TypeElement(25), TypeElement(26)};

	for(size_t X = 0; X < 3; ++X)
	{
		for(size_t Y = 0; Y < 3; ++Y)
		{
			for(size_t Z = 0; Z < 3; ++Z)
			{
				SurgSim::Math::Vector3d offset(this->m_size[0] * X, this->m_size[1] * Y, this->m_size[2] * Z);
				grid.addElement(element[number.toDecimal()], position + offset);
				number.next();
			}
		}
	}

	ASSERT_EQ(27u, grid.getActiveCells().size()); ///< 27 cells
	ASSERT_EQ(27u, grid.getCellIds().size()); ///< 27 elements
	for (size_t elementId = 0; elementId < 27; elementId++)
	{
		ASSERT_NO_THROW(grid.getCellIds().at(element[elementId]));
		for (size_t otherElementId = 0; otherElementId < 27; otherElementId++)
		{
			if (elementId == otherElementId)
			{
				continue;
			}
			/// Each cell contains only 1 unique element and each element is associated to a unique cell
			ASSERT_NE(grid.getCellIds()[element[elementId]], grid.getCellIds()[element[otherElementId]]);
		}
	}

	for (size_t elementId = 0; elementId < 27; elementId++)
	{
		ASSERT_NO_THROW(grid.getNeighbors(element[elementId]));
		ASSERT_GT(grid.getNeighbors(element[elementId]).size(), 0u);
	}

	// The central element should have all the elements for neighbors (including itself)
	auto& e111Neighbors = grid.getNonConstNeighbors(element[13]);
	ASSERT_EQ(27u, e111Neighbors.size());
	for (size_t elementId = 0; elementId < 27; elementId++)
	{
		auto found = std::find(e111Neighbors.begin(), e111Neighbors.end(), element[elementId]);
		ASSERT_NE(e111Neighbors.end(), found);
	}
}

TYPED_TEST(GridTestBase, ResetTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->validPowerOf2Dimension);

	TypeElement e0(0), e1(1);
	grid.addElement(e0, Eigen::Matrix<double, TestFixture::dimension, 1>::Zero());
	grid.addElement(e1, Eigen::Matrix<double, TestFixture::dimension, 1>::Zero());
	ASSERT_NE(0u, grid.getActiveCells().size());
	ASSERT_NE(0u, grid.getCellIds().size());
	ASSERT_NE(0u, grid.getNeighbors(e0).size());
	ASSERT_NE(0u, grid.getNeighbors(e1).size());

	ASSERT_NO_THROW(grid.reset());
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());
	ASSERT_EQ(0u, grid.getNeighbors(e0).size());
	ASSERT_EQ(0u, grid.getNeighbors(e1).size());
}

}; // namespace DataStructures
}; // namespace SurgSim
