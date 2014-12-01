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
	MockGrid(double size, const Eigen::Matrix<size_t, N, 1>& exponents) : Grid<T,N>(size, exponents){}

	std::unordered_map<size_t, std::vector<T>>& getActiveCells() { return this->m_activeCells; }

	std::unordered_map<T, size_t>& getCellIds() { return this->m_cellIds; }

	std::unordered_map<T, std::vector<T>>& getNonConstNeighborsMap() { return this->m_neighbors; }

	double getSize() const { return this->m_size; }

	Eigen::Matrix<size_t, N, 1> getNumCells() const { return this->m_numCells; }

	Eigen::Matrix<size_t, N, 1> getExponents() const { return this->m_exponents; }

	Eigen::Matrix<size_t, N, 1> getOffsets() const { return this->m_offsets; }

	Eigen::Matrix<size_t, N, 1> getOffsetExponents() const { return this->m_offsetExponents; }

	Eigen::AlignedBox<double, N> getAABB() const { return this->m_aabb; }
};

}; // namespace Particles

}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_UNITTESTS_MOCKOBJECT_H
