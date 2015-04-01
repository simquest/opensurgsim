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

#include "SurgSim/Math/Vector.h"
#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/DataStructures/UnitTests/GridTests.h"

namespace SurgSim
{
namespace DataStructures
{

template <typename T>
class Grid1DTestBase : public GridTestBase<T, 1>
{
public:
	static const size_t dimension = 1u;
	typedef T TypeElement;
};

typedef ::testing::Types<size_t, int, float, double, ElementTest> MyTypes;
TYPED_TEST_CASE(Grid1DTestBase, MyTypes);

TYPED_TEST(Grid1DTestBase, ConstructorTest)
{
	typedef typename TestFixture::GridType GridType;

	ASSERT_NO_THROW({GridType grid(this->m_size, this->m_aabb1Cell);});
	ASSERT_NO_THROW({GridType grid(this->m_size, this->m_aabb);});

	GridType grid(this->m_size, this->m_aabb);
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());
	ASSERT_TRUE(grid.getSize().isApprox(this->m_size));
	ASSERT_TRUE(grid.getAABB().isApprox(this->m_aabb));
}

TYPED_TEST(Grid1DTestBase, addElementTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->m_aabb);

	// Add an element outside of the grid
	auto positionMin =  this->m_aabb.max() - this->m_aabb.sizes() * 1.001;
	EXPECT_FALSE(grid.addElement(TypeElement(), positionMin));
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());

	// Add an element outside of the grid
	auto positionMax =  this->m_aabb.min() + this->m_aabb.sizes() * 1.001;
	EXPECT_FALSE(grid.addElement(TypeElement(), positionMax));
	ASSERT_EQ(0u, grid.getActiveCells().size());
	ASSERT_EQ(0u, grid.getCellIds().size());

	// Add an element inside of the grid
	TypeElement e0(0), e1(1);
	EXPECT_TRUE(grid.addElement(e0, Eigen::Matrix<double, TestFixture::dimension, 1>::Zero()));
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(1u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_ANY_THROW(grid.getCellIds().at(e1));

	// Add an element inside of the grid, in the same cell
	EXPECT_TRUE(grid.addElement(e1, Eigen::Matrix<double, TestFixture::dimension, 1>::Zero()));
	ASSERT_EQ(1u, grid.getActiveCells().size());
	ASSERT_EQ(2u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);

	// Add an element inside of the grid, in a different cell
	TypeElement e2(2);
	EXPECT_TRUE(grid.addElement(e2, this->m_size * 1.5));
	ASSERT_EQ(2u, grid.getActiveCells().size());
	ASSERT_EQ(3u, grid.getCellIds().size());
	ASSERT_NO_THROW(grid.getCellIds().at(e0));
	ASSERT_NO_THROW(grid.getCellIds().at(e1));
	ASSERT_NO_THROW(grid.getCellIds().at(e2));
	ASSERT_EQ(grid.getCellIds()[e0], grid.getCellIds()[e1]);
	ASSERT_NE(grid.getCellIds()[e0], grid.getCellIds()[e2]);
	ASSERT_NE(grid.getCellIds()[e1], grid.getCellIds()[e2]);
}

TYPED_TEST(Grid1DTestBase, NeighborsTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->m_aabb);

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

TYPED_TEST(Grid1DTestBase, ResetTest)
{
	typedef typename TestFixture::GridType GridType;
	typedef typename TestFixture::TypeElement TypeElement;

	GridType grid(this->m_size, this->m_aabb);

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
