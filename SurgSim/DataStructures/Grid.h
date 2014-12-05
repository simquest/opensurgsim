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

#ifndef SURGSIM_DATASTRUCTURES_GRID_H
#define SURGSIM_DATASTRUCTURES_GRID_H

#include <array>
#include <unordered_map>

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace DataStructures
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
/// This grid is optimized to contain 2^exponents[i] number of cells per dimension and is
/// formed of cells of size 'size':
/// from -0.5 * size[dimension] * pow(2, exponents[dimension])
/// to    0.5 * size[dimension] * pow(2, exponents[dimension])
/// \tparam T Element type to be stored
/// \tparam N The dimension of the grid (i.e. 2 => 2D, 3 => 3D)
template <typename T, size_t N>
class Grid
{
public:
	/// Constructor
	/// \param cellSize The size of each cell in dimension N (i.e. cells are not necessarily cubic).
	/// \param exponents For each cell the exponent to 2 indicating the requested number of cell per dimension,
	///        the sum of all the exponents can't exceed the architecture bit size i.e. 32 and 64 respectively
	/// \note Power of 2 limitation:<br>
	/// \note * 32 bit architecture: maximum exponents sum is 32, for example in 3d {{10, 10, 10}}<br>
	/// \note * 64 bit architecture: maximum exponents sum is 64, for example in 3d {{21, 21, 21}}
	Grid(const Eigen::Matrix<double, N, 1>& cellSize, const Eigen::Matrix<size_t, N, 1>& exponents);

	/// Destructor
	virtual ~Grid();

	/// Reset the grid content and the neighbors' mapping
	void reset();

	/// Add an element in the grid
	/// \param element to be added at this position
	/// \param position of the element in the n-D space
	/// \note If the position is outside of the grid, the element is simply not added to the grid
	template <class Derived>
	void addElement(const T element, const Eigen::MatrixBase<Derived>& position);

	/// Retrieve an elements' neighbors
	/// \param element The element for which the neighbors are requested
	/// \return The element's neighbors list (including the element itself)
	const std::vector<T>& getNeighbors(const T& element);

protected:
	/// Data structure for a cell's content (the list of elements and the list of all the neighbors)
	typedef struct
	{
		std::vector<T> elements;
		std::vector<T> neighbors;
	} CellContent;

	/// Active cells (referenced by their ids (spatial hashing)) with their content
	std::unordered_map<size_t, CellContent> m_activeCells;

	/// Mapping from element to cell id containing the element
	std::unordered_map<T, size_t> m_cellIds;

	/// Size of each cell (same on all dimension)
	Eigen::Matrix<double, N, 1> m_size;

	/// Number of cells per dimension
	Eigen::Matrix<size_t, N, 1> m_numCells;

	/// Exponent of 2 cells per dimension
	Eigen::Matrix<size_t, N, 1> m_exponents;

	/// Offset (in exponent of 2) for each dimension to go from n-d array to 1d array and vice-versa
	Eigen::Matrix<size_t, N, 1> m_offsetExponents;

	/// Grid min and max
	Eigen::AlignedBox<double, N> m_aabb;

	/// Does the neighbors needs to be recomputed ?
	bool m_neighborsDirtyFlag;

private:
	/// Update the neighbors lists
	void update();

	/// Retrieve the neighboring cells id (including this cell)
	/// \param cellId for which the neighbors cells are requested
	/// \param cellsIds [out] Neighbors cells ids (only if a cell is valid, undefined otherwise)
	/// \param cellsValidities [out] Neighbors cells validity (True if a cell exists in the grid, false otherwise)
	void getNeighborsCellIds(size_t cellId,
		Eigen::Matrix<size_t, powerOf3<N>::value, 1>* cellsIds,
		Eigen::Matrix<bool, powerOf3<N>::value, 1>* cellsValidities);

	/// Cell id mapping from dimension-N to dimension-1 (without input validity check)
	/// \param nd The cell id in dimension-N
	/// \return The cell id in dimension-1
	/// \note No check is performed on the validity of the input cell id.
	/// \note Return value undefined if invalid input
	size_t mapNdTo1d(const Eigen::Matrix<int, N, 1>& nd) const;

	/// Cell id mapping from dimension-N to dimension-1 (with input validity check)
	/// \param nd The cell id in dimension-N
	/// \param [out] oned The cell id in dimension-1 if valid is True, undefined otherwise
	/// \param [out] valid The cell validity (grid spatial limit check)
	void mapNdTo1d(const Eigen::Matrix<int, N, 1>& nd, size_t* oned, bool* valid) const;

	/// Cell id mapping from dimension-1 to dimension-N (without input validity check)
	/// \param oned A dimension-1 id
	/// \param [out] nd The cell dimension-d id
	void map1dToNd(size_t oned, Eigen::Matrix<int, N, 1>* nd) const;
};

};  // namespace DataStructures
};  // namespace SurgSim

#include "SurgSim/DataStructures/Grid-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_GRID_H
