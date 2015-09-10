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

/// n-dimensional grid structure with uniform non-cubic cells
/// This data structure is useful to search for neighbors in a given range (the size of each cell)
/// \tparam T Element type to be stored
/// \tparam N The dimension of the grid (i.e. 2 => 2D, 3 => 3D)
template <typename T, size_t N>
class Grid
{
public:
	/// Constructor
	/// \param cellSize The size of each cell in dimension N (i.e. cells are not necessarily cubic).
	/// \param bounds The dimension-N boundaries of the space covered by the grid.
	Grid(const Eigen::Matrix<double, N, 1>& cellSize, const Eigen::AlignedBox<double, N>& bounds);

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

	/// Retrieve the neighbors of a location
	/// \param position The position for which the neighbors are requested
	/// \return The neighbors for this position, i.e. all the elements in the positions cell and all surrounding cells
	template <class Derived>
	const std::vector<T>& getNeighbors(const Eigen::MatrixBase<Derived>& position);

protected:
	/// Data structure for a cell's content (the list of elements and the list of all the neighbors)
	typedef struct
	{
		std::vector<T> elements;
		std::vector<T> neighbors;
	} CellContent;

	/// The type of the n-dimensional cell Id.
	typedef Eigen::Matrix<int, N, 1> NDId;

	/// Enable the NDId to be used as a key in an unordered map.
	class NDIdHash
	{
	public:
		size_t operator()(const NDId& nd) const;
	};

	/// Active cells (referenced by their ids (spatial hashing)) with their content
	std::unordered_map<NDId, CellContent, NDIdHash> m_activeCells;

	/// Mapping from element to cell id containing the element
	std::unordered_map<T, NDId> m_cellIds;

	/// Size of each cell (same on all dimension)
	Eigen::Matrix<double, N, 1> m_size;

	/// Grid min and max
	Eigen::AlignedBox<double, N> m_aabb;

	/// Does the neighbors needs to be recomputed ?
	bool m_neighborsDirtyFlag;

private:
	/// Update the neighbors lists
	void update();

	/// Retrieve the neighboring cells id (including this cell)
	/// \param cellId for which the neighbors cells are requested
	/// \param cellsIds [out] Neighbors cells ids
	void getNeighborsCellIds(NDId cellId,
		std::array<NDId, powerOf3<N>::value>* cellsIds);

	/// \param a The first cell ID.
	/// \param b The second cell ID.
	/// \return true if a is > b.
	bool isNdGreaterOrEqual(const NDId& a, const NDId& b);
};

};  // namespace DataStructures
};  // namespace SurgSim

#include "SurgSim/DataStructures/Grid-inl.h"

#endif  // SURGSIM_DATASTRUCTURES_GRID_H
