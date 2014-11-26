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

#ifndef SURGSIM_PARTICLES_GRID_H
#define SURGSIM_PARTICLES_GRID_H

#include <array>
#include <unordered_map>

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Particles
{

/// Templated function to compute a power of 3 at compile time (useful for template parameter)
template <size_t N>
struct powerOf3
{
	enum { value = 3 * powerOf3<N-1>::value };
};
template <>
struct powerOf3<0>
{
	enum { value = 1 };
};

/// n-dimension grid structure with a fixed uniform size on all directions
/// This data structure is useful to search for neighbors in a given range (the size of each cell)
/// This grid is optimized to contain a power of 2 number of cells on each dimension.
/// The grid is formed of cubic cells of size 'size' extending on each dimension
/// from -0.5 * size * pow(2, powerOf2CellsPerDimension[dimension])
/// to    0.5 * size * pow(2, powerOf2CellsPerDimension[dimension])
/// \tparam T Element type to be stored
/// \tparam N The dimension of the grid (i.e. 2 => 2D, 3 => 3D)
template <typename T, size_t N>
class Grid
{
public:
	/// Constructor
	Grid();

	/// Destructor
	virtual ~Grid();

	/// Initialization
	/// \param size The size of each cell, should be the range in which we want to look for neighbors
	/// \param powerOf2CellsPerDimension The power of 2 cells per dimension (i.e. 8 => 2^8 cells)
	/// \note Power of 2 limitation:<br>
	/// \note * 32 bit architecture: maximum number of cells is 2^32, for example in 3d {{2^10, 2^10, 2^10}}<br>
	/// \note * 64 bit architecture: maximum number of cells is 2^64, for example in 3d {{2^21, 2^21, 2^21}}
	void init(double size, const Eigen::Matrix<size_t, N, 1>& powerOf2CellsPerDimension);

	/// Reset the grid content and the neighbors' mapping
	void reset();

	/// Add an element in the grid
	/// \param position of the element in the n-D space
	/// \param element to be added at this position
	/// \note If the position is outside of the grid, the element is simply not added to the grid
	template <class Derived>
	void addElementAt(const Eigen::MatrixBase<Derived>& position, const T element);

	/// Compute all the particles' neighbors list
	/// \note The neighbors lists are inclusive (the element itself is in its neighbor's list)
	/// \note The neighbors lists are cleared on reset call
	void computeNeighborsMap();

	/// Retrieve the elements' neighbors mapping
	/// \return The mapping from any element to its corresponding neighbors elements within neighbors cells
	/// \note The neighbors lists are inclusive (the element itself is in its neighbor's list)
	const std::unordered_map<T, std::vector<T>>& getNeighborsMap() const;

protected:
	/// Active cells (referenced by their ids) with their content
	std::unordered_map<size_t, std::vector<T>> m_activeCells;

	/// Mapping from element to cell id containing the element
	std::unordered_map<T, size_t> m_mapElementToCellId;

	/// Mapping from element to element's neighbors
	std::unordered_map<T, std::vector<T>> m_mapElementNeighbors;

	/// Size of each cell (same on all dimension)
	double m_size;

	/// Number of cells per dimension
	Eigen::Matrix<size_t, N, 1> m_numCellsPerDimension;

	/// Power of 2 cells per dimension
	Eigen::Matrix<size_t, N, 1> m_powerOf2CellsPerDimension;

	/// Offset for each dimension to go from n-d array to 1d array and vice-versa
	Eigen::Matrix<size_t, N, 1> m_offsetPerDimension;

	/// Offset (in power of 2) for each dimension to go from n-d array to 1d array and vice-versa
	Eigen::Matrix<size_t, N, 1> m_offsetPowerOf2PerDimension;

	/// Total number of cells = product[over dimension](m_numCellsPerDimension[i])
	size_t m_numCells;

	/// Grid min and max
	Eigen::AlignedBox<double, N> m_aabb;

private:
	/// Compute the neighbors' list for a given element
	/// \param element for which the neighbors are requested
	/// \param [out] result The element neighbors
	/// \note The list also include the element itself
	void getNeighborsOfElement(const T& element, std::vector<T>* result);

	/// Retrieve the neighboring cells id (excluding this cell)
	/// \param cellId for which the neighbors cells are requested
	/// \param cellsId [out] Neighbors cells ids (only if a cell is valid, undefined otherwise)
	/// \param cellsValidity [out] Neighbors cells validity (True if a cell exists in the grid, false otherwise)
	void getNeighborsCellId(size_t cellId,
		Eigen::Matrix<size_t, powerOf3<N>::value, 1>* cellsId,
		Eigen::Matrix<bool, powerOf3<N>::value, 1>* cellsValidity);

	/// Cell id correspondence from dimension-D to 1d (without input validity check)
	/// \param cellIdnD The cell id in dimension-D
	/// \return The cell unique 1d id
	/// \note No check is performed on the validity of the input cell id.
	/// \note Return value undefined if invalid input
	size_t cellId_ndTo1d(const Eigen::Matrix<int, N, 1>& cellIdnD) const;

	/// Cell id correspondence from dimension-D to 1d (with input validity check)
	/// \param cellIdnD The cell id in dimension-D
	/// \param [out] cellId1D The cell unique 1d id if valid is True, undefined otherwise
	/// \param [out] valid The cell validity (existence in the grid)
	void cellId_ndTo1d(const Eigen::Matrix<int, N, 1>& cellIdnD, size_t* cellId1D, bool* valid) const;

	/// Cell id correspondence from 1d to dimension-D (without input validity check)
	/// \param cellId1D A cell unique 1d id
	/// \param [out] cellIdnD The cell dimension-d id
	void cellId_1dTond(size_t cellId1D, Eigen::Matrix<int, N, 1>* cellIdnD) const;
};

};  // namespace Particles
};  // namespace SurgSim

#include "SurgSim/Particles/Grid-inl.h"

#endif  // SURGSIM_PARTICLES_GRID_H
