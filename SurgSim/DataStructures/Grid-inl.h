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

#ifndef SURGSIM_DATASTRUCTURES_GRID_INL_H
#define SURGSIM_DATASTRUCTURES_GRID_INL_H

#include <boost/functional/hash.hpp>

namespace SurgSim
{
namespace DataStructures
{

namespace
{

/// Class handling number in a given base with a given number of digits
/// Example numbers in base 3 with 3 digits are defined in order:
/// 000, 001, 002, 010, 011, 012, 020, 021, 022,
/// 100, 101, 102, 110, 111, 112, 120, 121, 122
/// 200, 201, 202, 210, 211, 212, 220, 221, 222
/// Note that the storage is done in Eigen column-major vector (ith entry corresponds to the ith degree of the base)
/// Expected usage looking for all neighbors in a 3d array:
///    Number<size_t, 3, 3> neighborOffset;
///    Eigen::Matrix<size_t, 3, 1> currentElement = ...; // Location of an element in a 3d grid;
///    do{
///      Eigen::Matrix<size_t, 3, 1> neighborElement = currentElement + neighborOffset;
///      // do something with the neighbor element
/// ...} while (neighborOffset.next());
/// \tparam T The type storing the number (must be an integral type)
/// \tparam B The base in which the number is expressed (must be within [2..9])
/// \tparam N The number of digits for this number
template <typename T, size_t B, size_t N>
class Number : public Eigen::Matrix<T, N, 1>
{
public:
	static_assert(B > 1 && B < 10, "B (the base) needs to be within [2..9]");

	/// Constructor
	Number()
	{
		this->setZero();
	}

	/// \return the number expressed in base 10
	size_t toDecimal() const
	{
		size_t value = 0;
		size_t BexponentDigit = 1;
		for (size_t digit = 0; digit < N; ++digit)
		{
			value += (*this)[digit] * BexponentDigit;
			BexponentDigit *= B;
		}
		return value;
	}

	/// Increment the number
	/// \return False if there is no next number on N digits, True otherwise.
	bool next()
	{
		size_t digit = 0;
		do
		{
			(*this)[digit]++;
			if ((*this)[digit] == B)
			{
				(*this)[digit] = 0;
			}
			else
			{
				return true;
			}
			digit++;
		}
		while (digit < N);

		return false;
	}
};
}; // namespace


template <typename T, size_t N>
size_t Grid<T, N>::NDIdHash::operator()(const NDId& nd) const
{
	return boost::hash_range(nd.data(), nd.data() + N);
}

template <typename T, size_t N>
Grid<T, N>::Grid(const Eigen::Matrix<double, N, 1>& cellSize, const Eigen::AlignedBox<double, N>& bounds)
	: m_size(cellSize),
	m_aabb(bounds),
	m_neighborsDirtyFlag(false)
{
	static_assert(N >= 1, "A grid must have a positive non null dimension");
}

template <typename T, size_t N>
Grid<T, N>::~Grid()
{
}

template <typename T, size_t N>
void Grid<T, N>::reset()
{
	// Clear the mapping element -> cellId
	m_cellIds.clear();

	// Clear the active cells
	m_activeCells.clear();

	// Nothing in the grid (no elements, no neighbors)...so it is up to date
	m_neighborsDirtyFlag = false;
}

template <typename T, size_t N>
template <class Derived>
void Grid<T, N>::addElement(const T element, const Eigen::MatrixBase<Derived>& position)
{
	// Only add element that are located in the grid
	if (!m_aabb.contains(position))
	{
		return;
	}

	// Find the dimension-N cell id from the location
	// Example in 3D: cell (i, j, k) has 3D min/max coordinates
	//   min[axis] = (size[axis] * (-numCellPerDim[axis] / 2 + i)
	//   max[axis] = (size[axis] * (-numCellPerDim[axis] / 2 + i + 1)
	NDId cellId = ((position - m_aabb.min()).cwiseQuotient(m_size)).template cast<int>();

	// Register the element into its corresponding cell if it exists, or creates it.
	m_activeCells[cellId].elements.push_back(element);

	// Add this element in the map [element -> cellID]
	m_cellIds[element] = cellId;

	/// Flag that the neighbors list will need to be recomputed on the next access
	m_neighborsDirtyFlag = true;
}

template <typename T, size_t N>
void Grid<T, N>::update()
{
	std::array<NDId, powerOf3<N>::value> cellsIds;

	m_positionCache.clear();

	// Start by clearing up all the neighbor's list
	for (typename std::unordered_map<NDId, typename Grid<T, N>::CellContent, NDIdHash>::reference cell : m_activeCells)
	{
		cell.second.neighbors.clear();
	}

	// Compute each cell's neighbors list
	for (typename std::unordered_map<NDId, typename Grid<T, N>::CellContent, NDIdHash>::reference cell : m_activeCells)
	{
		getNeighborsCellIds(cell.first, &cellsIds);

		for (size_t index = 0; index < powerOf3<N>::value; ++index)
		{
			// Use symmetry between pair of cells to only treat neighbors with a larger or equal id.
			if (isNdGreaterOrEqual(cellsIds[index], cell.first))
			{
				auto neighborCell = m_activeCells.find(cellsIds[index]);
				if (neighborCell != m_activeCells.end())
				{
					cell.second.neighbors.insert(cell.second.neighbors.end(),
						neighborCell->second.elements.cbegin(), neighborCell->second.elements.cend());

					// Treat symmetry if the cells are different
					if (cellsIds[index] != cell.first)
					{
						neighborCell->second.neighbors.insert(neighborCell->second.neighbors.end(),
							cell.second.elements.cbegin(), cell.second.elements.cend());
					}
				}
			}
		}
	}
}

template <typename T, size_t N>
const std::vector<T>& Grid<T, N>::getNeighbors(const T& element)
{
	static std::vector<T> empty;

	if (m_neighborsDirtyFlag)
	{
		update();
		m_neighborsDirtyFlag = false;
	}

	auto const foundCell = m_cellIds.find(element);
	if (foundCell != m_cellIds.cend())
	{
		return m_activeCells[foundCell->second].neighbors;
	}

	return empty;
}

template <typename T, size_t N>
template <class Derived>
const std::vector<T>& Grid<T, N>::getNeighbors(const Eigen::MatrixBase<Derived>& position)
{
	const static std::vector<T> empty;

	// If outside the bounding box, can't find any neighbors
	if (m_aabb.contains(position))
	{
		if (m_neighborsDirtyFlag)
		{
			update();
			m_neighborsDirtyFlag = false;
		}

		NDId cellId = ((position - m_aabb.min()).cwiseQuotient(m_size)).template cast<int>();

		auto foundCell = m_activeCells.find(cellId);

		// If position is in a cell that exists, return that cells neighbors,
		if (foundCell != m_activeCells.end())
		{
			return (foundCell->second).neighbors;
		}
		else
		{
			// If the neighbors are in the cache return them
			// otherwise calculate all the neighbors
			auto foundPosition = m_positionCache.find(cellId);
			if (foundPosition == m_positionCache.end())
			{
				std::array<NDId, powerOf3<N>::value> cellsIds;
				getNeighborsCellIds(cellId, &cellsIds);
				std::vector<T> neigbors;
				for (const auto& neighborId : cellsIds)
				{
					auto neighborCell = m_activeCells.find(neighborId);
					if (neighborCell != m_activeCells.end())
					{
						neigbors.insert(neigbors.end(),
										neighborCell->second.elements.cbegin(),
										neighborCell->second.elements.cend());
					}
				}
				m_positionCache[cellId] = std::move(neigbors);
			}
			return m_positionCache[cellId];
		}
	}
	return empty;
}

template <typename T, size_t N>
void Grid<T, N>::getNeighborsCellIds(NDId cellId,
									 std::array<NDId, powerOf3<N>::value>* cellsIds)
{
	// Now build up all the 3^N neighbors cell around this n-d cell
	// It corresponds to all possible permutation in dimension-N of the indices
	// {(cellIdnD[0] - 1, cellIdnD[0], cellIdnD[0] + 1) x ... x
	//  (cellIdnD[N - 1] - 1, cellIdnD[N - 1], cellIdnD[N - 1] + 1)}
	// It is (cellIdnD[0] - 1, ... , cellIdnD[N - 1] - 1) + all possible number in base 3 with N digit
	// For example in 2D, the NumberInBase3<2> are in order: 00 01 02 10 11 12 20 21 22
	// For example in 3D, the NumberInBase3<3> are in order:
	//   000 001 002 010 011 012 020 021 022
	//   100 101 102 110 111 112 120 121 122
	//   200 201 202 210 211 212 220 221 222
	cellId -= NDId::Ones();

	Number<int, 3, N> currentNumberNDigitBase3;
	for (size_t i = 0; i < powerOf3<N>::value; ++i)
	{
		(*cellsIds)[i] = cellId + currentNumberNDigitBase3;
		currentNumberNDigitBase3.next();
	}
}

template <typename T, size_t N>
bool Grid<T, N>::isNdGreaterOrEqual(const NDId& a, const NDId& b)
{
	for (size_t i = 0; i < N; ++i)
	{
		if (a[i] > b[i])
		{
			return true;
		}

		if (a[i] < b[i])
		{
			return false;
		}
	}
	return true;
}

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_GRID_INL_H
