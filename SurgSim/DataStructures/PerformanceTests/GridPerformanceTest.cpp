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

#include <array>

#include "SurgSim/Framework/Timer.h"
#include "SurgSim/DataStructures/Grid.h"

namespace SurgSim
{
namespace DataStructures
{

class GridPerformanceTestBase : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		// The various concentration of elements per axis
		size_t concentrationPerAxisId = 0;

		// Fine discretization of the concentration per cell from 1^3 to 3^3
		for (double concentrationPerCell = pow(1.0, 3.0); concentrationPerCell <= pow(3.0, 3.0); concentrationPerCell++)
		{
			double concentrationPerAxis = pow(concentrationPerCell, 1.0 / 3.0);
			m_elementsPerAxis[concentrationPerAxisId++] = concentrationPerAxis;
		}
		// Coarse discretization of the concentration per cell from 4^3 to 10^3
		for (double concentrationPerAxis = 4.0; concentrationPerAxis <= 10.0; concentrationPerAxis++)
		{
			m_elementsPerAxis[concentrationPerAxisId++] = concentrationPerAxis;
		}

		m_h = 0.1;
		m_bounds.min().setConstant(-pow(2, 10) / 2.0);
		m_bounds.max().setConstant(pow(2, 10) / 2.0);
		m_grid = std::make_shared<Grid<size_t, 3>>(Eigen::Matrix<double, 3, 1>::Constant(m_h), m_bounds);
	}

	void addElementsUniformDistribution(const Eigen::Matrix<size_t, 3, 1>& numElementsPerAxis,
										double concentrationPerAxis)
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
				//printf("(%lu %lu)",x,y);
			}
			//printf("\n");
		}
	}

	void performTimingTest()
	{
		SurgSim::Framework::Timer timer;

		printf("  Notation: e. stands for elements\n");
		printf("  Unit: Concentration is the number of elements per cell\n");
		printf("  Unit: Times are given in seconds\n");
		printf("-------------------------------------------------------------------------------\n");
		printf("Concentration | 50^3e.  60^3e.  70^3e.  80^3e.  90^3e.  100^3e. 110^3e. 120^3e.\n");
		printf("-------------------------------------------------------------------------------\n");
		for (double concentrationPerAxis : m_elementsPerAxis)
		{
			printf("%7.2lf       | ", concentrationPerAxis * concentrationPerAxis * concentrationPerAxis);
			for (size_t i = 5; i <= 12; i++)
			{
				timer.start();

				// Clear the grid from all previously added elements and clear the neighbor's list
				m_grid->reset();

				// Add all elements in the grid, triggering a dirty flag for the neighbor's list
				addElementsUniformDistribution(Eigen::Matrix<size_t, 3, 1>::Constant(i * 10), concentrationPerAxis);

				// Request any neighbor's list to force all neighbor's lists recalculation
				m_grid->getNeighbors(0);

				timer.endFrame();

				printf("%6.3lf  ", timer.getLastFramePeriod());
				fflush(stdout);
			}
			printf("\n");
		}
		printf("-------------------------------------------------------------------------------\n");
	}

protected:
	/// Concentration of elements per axis to be tested
	std::array<double, 34> m_elementsPerAxis;

	/// Grid size (cells are cubic in this test)
	double m_h;

	/// Grid boundary
	Eigen::AlignedBox<double, 3> m_bounds;

	/// Grid
	std::shared_ptr<Grid<size_t, 3>> m_grid;
};

TEST_F(GridPerformanceTestBase, GridTest)
{
	performTimingTest();
}

} // namespace DataStructures
} // namespace SurgSim
