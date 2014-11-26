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

#ifndef SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECT_H
#define SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECT_H

#include "SurgSim/Particles/Grid.h"

namespace SurgSim
{

namespace Particles
{

template <typename T, size_t N>
class MockGrid : public Grid<T, N>
{
public:
	MockGrid() : Grid<T,N>(){}

	std::unordered_map<size_t, std::vector<T>>& getActiveCells() { return m_activeCells; }

	std::unordered_map<T, size_t>& getMappingElementToCellId() { return m_mapElementToCellId; }

	std::unordered_map<T, std::vector<T>>& getNonConstNeighborsMap() { return m_mapElementNeighbors; }

	double getSize() const { return m_size; }

	Eigen::Matrix<size_t, N, 1> getNumCellsPerDimension() const { return m_numCellsPerDimension; }

	Eigen::Matrix<size_t, N, 1> getPowerOf2CellsPerDimension() const { return m_powerOf2CellsPerDimension; }

	Eigen::Matrix<size_t, N, 1> getOffsetPerDimension() const { return m_offsetPerDimension; }

	Eigen::Matrix<size_t, N, 1> getOffsetPowerOf2PerDimension() const { return m_offsetPowerOf2PerDimension; }

	size_t getNumCells() const { return m_numCells; }

	Eigen::AlignedBox<double, N> getAABB() const { return m_aabb; }
};

}; // namespace Particles

}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECT_H
