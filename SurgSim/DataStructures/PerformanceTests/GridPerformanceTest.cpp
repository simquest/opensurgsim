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

#include <boost/exception/to_string.hpp>

#include <array>

#include "SurgSim/Framework/Timer.h"
#include "SurgSim/DataStructures/Grid.h"

namespace SurgSim
{
namespace DataStructures
{

/// This class test the grid timings for a given concentration of elements per cell and a given number of element per
/// dimension. These two information are embedded in GTest WithParamInterface which takes a
/// tuple<double = concentrationPerCell, size_t = numElementsPerDimension>
class Grid3DPerformanceTests : public ::testing::Test,
							   public ::testing::WithParamInterface<std::tuple<double, size_t>>
{
public:
	typedef Eigen::Matrix<size_t, 3, 1> Vector3ui;

	virtual void SetUp()
	{
		m_h = 0.1;
		m_bounds.min().setConstant(-pow(2, 10) / 2.0);
		m_bounds.max().setConstant(pow(2, 10) / 2.0);
		m_grid = std::make_shared<Grid<size_t, 3>>(Eigen::Matrix<double, 3, 1>::Constant(m_h), m_bounds);
	}

	void addElementsUniformDistribution(const Vector3ui& numElementsPerAxis, double concentrationPerAxis)
	{
		double coef = m_h / static_cast<double>(concentrationPerAxis);
		size_t elementId = 0;

		for (size_t x = 0; x < numElementsPerAxis[0]; x++)
		{
			for (size_t y = 0; y < numElementsPerAxis[1]; y++)
			{
				for (size_t z = 0; z < numElementsPerAxis[2]; z++)
				{
					SurgSim::Math::Vector3d point(x * coef, y * coef, z * coef);
					m_grid->addElement(elementId, point);
					elementId++;
				}
			}
		}
	}

	double performTimingTest(double concentrationPerCell, size_t numElementPerDimension)
	{
		SurgSim::Framework::Timer timer;
		Vector3ui numElementsPerAxis = Vector3ui::Constant(numElementPerDimension);
		double concentrationPerAxis = pow(concentrationPerCell, 1.0 / 3.0);

		timer.start();

		// Clear the grid from all previously added elements and clear the neighbor's list
		m_grid->reset();
		// Add all elements in the grid, triggering a dirty flag for the neighbor's list
		addElementsUniformDistribution(numElementsPerAxis, concentrationPerAxis);
		// Request any neighbor's list to force all neighbor's lists recalculation
		m_grid->getNeighbors(0);

		timer.endFrame();

		return timer.getCumulativeTime();
	}

protected:
	/// Grid size (cells are cubic in this test)
	double m_h;

	/// Grid boundary
	Eigen::AlignedBox<double, 3> m_bounds;

	/// Grid
	std::shared_ptr<Grid<size_t, 3>> m_grid;
};

TEST_P(Grid3DPerformanceTests, Grid3DTest)
{
	double concentrationPerCell;
	size_t numElementsPerDimension;
	std::tie(concentrationPerCell, numElementsPerDimension) = GetParam();
	size_t numElements = numElementsPerDimension * numElementsPerDimension * numElementsPerDimension;
	RecordProperty("Concentration of elements per cell", boost::to_string(concentrationPerCell));
	RecordProperty("Total number of elements", boost::to_string(numElements));
	RecordProperty("Time (in s)", boost::to_string(performTimingTest(concentrationPerCell, numElementsPerDimension)));
}

INSTANTIATE_TEST_CASE_P(
	Grid3D,
	Grid3DPerformanceTests,
	::testing::Combine(
		// Concentration per cell is fine between 1 and 3^3, then coarser
		::testing::Values(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0,
						  18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0,
						  4 * 4 * 4, 5 * 5 * 5, 6 * 6 * 6, 7 * 7 * 7, 8 * 8 * 8, 9 * 9 * 9, 10 * 10 * 10),
		// Number of elements per dimension
		::testing::Values(50, 60, 60, 70, 80, 90, 100, 110, 120)));

} // namespace DataStructures
} // namespace SurgSim
