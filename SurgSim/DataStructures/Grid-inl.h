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

namespace SurgSim
{
namespace DataStructures
{

namespace
{

/// Class handling number in a given base with a given number of digit
/// Example numbers in base 3 with 3 digits are defined in order:
/// 000, 001, 002, 010, 011, 012, 020, 021, 022,
/// 100, 101, 102, 110, 111, 112, 120, 121, 122
/// 200, 201, 202, 210, 211, 212, 220, 221, 222
/// Note that the storage is done in Eigen column-major vector (ith entry corresponds to the ith degree of the base)
/// \tparam T The type storing the number (must be an integral type)
/// \tparam B The base in which the number is expressed (must be within [2..9])
/// \tparam N The number of digit for this number
template <typename T, size_t B, size_t N>
class Number : public Eigen::Matrix<T, N, 1>
{
public:
	static_assert(B > 1 && B < 10, "B (the base) needs to be within [2..9]");

	Number()
	{
		this->setZero();
	}

	size_t toDecimal() const
	{
		size_t value = 0;
		size_t BPowerDim = 1;
		for (size_t dim = 0; dim < N; ++dim)
		{
			value += (*this)[dim] * BPowerDim;
			BPowerDim *= B;
		}
		return value;
	}

	bool next()
	{
		size_t dim = 0;
		do
		{
			(*this)[dim]++;
			if ((*this)[dim] == B)
			{
				(*this)[dim] = 0;
			}
			else
			{
				return true;
			}
			dim++;
		}
		while (dim < N);

		return false;
	}
};
}; // namespace

template <typename T, size_t N>
Grid<T, N>::Grid(double size, const Eigen::Matrix<size_t, N, 1>& exponents)
{
	// Check that the size of the requested grid fit the current architecture.
	size_t maximumCellsPowerOf2 = sizeof(size_t) * 8;
	size_t requestedCellsPozerOf2 = exponents.sum();
	SURGSIM_ASSERT(requestedCellsPozerOf2 <= maximumCellsPowerOf2) << "The requested grid dimension (2^"
		<< requestedCellsPozerOf2 << " cells) is too large for the "
		<< maximumCellsPowerOf2 << " bit architecture";

	m_size = size;
	m_exponents = exponents;
	for (size_t i = 0; i < N; ++i)
	{
		m_numCells[i] = (static_cast<size_t>(1u) << exponents[i]);
		m_aabb.min()[i] = -static_cast<double>(m_numCells[i]) * m_size * 0.5;
		m_aabb.max()[i] = static_cast<double>(m_numCells[i]) * m_size * 0.5;
	}

	// Cell indexing goes as follow:
	// Example: a cell in 3d with the indices (i, j, k) will have a 1d index of
	// i * m_offsetPerDimension[0] + j * m_offsetPerDimension[1] + k * m_offsetPerDimension[2]
	if (N >= 1)
	{
		m_offsets[N - 1] = 1;
		m_offsetExponents[N - 1] = 0;
	}
	for (int i = static_cast<int>(N) - 2; i >= 0; i--)
	{
		m_offsets[i] = m_offsets[i + 1] * m_numCells[i + 1];
		m_offsetExponents[i] = m_offsetExponents[i + 1] + m_exponents[i + 1];
	}
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
}

template <typename T, size_t N>
template <class Derived>
void Grid<T, N>::addElement(const T element, const Eigen::MatrixBase<Derived>& position)
{
	// Only add element that are located in the grid
	if (!(position.array() >= m_aabb.min().array() && position.array() <= m_aabb.max().array()).all())
	{
		return;
	}

	// Find the dimension-N cell id from the location
	// Example in 3D: cell (i, j, k) has 3D min/max coordinates
	//   min[axis] = (size * (-numCellPerDim[axis] / 2 + i)
	//   max[axis] = (size * (-numCellPerDim[axis] / 2 + i + 1)
	Eigen::Matrix<int, N, 1> cellIdnD;
	for (size_t i = 0; i < N; ++i)
	{
		double cellIdForThisDimension = position[i] / m_size + static_cast<double>(m_numCells[i] >> 1);
		cellIdnD[i] = static_cast<int>(floor(cellIdForThisDimension));
	}

	// Find the dimension-1 cell id from the dimension-N cell id
	size_t cellId1D = mappingNdTo1d(cellIdnD);

	// Register the element into its corresponding cell if it exists, or creates it.
	m_activeCells[cellId1D].elements.push_back(element);

	// Add this element in the map [element -> cellID]
	m_cellIds[element] = cellId1D;
}

template <typename T, size_t N>
void Grid<T, N>::update(void)
{
	Eigen::Matrix<size_t, powerOf3<N>::value, 1> cellsId;
	Eigen::Matrix<bool, powerOf3<N>::value, 1> cellsValidity;

	// Update each cell's neighbors list
	for (auto& cell : m_activeCells) // NOLINT
	{
		getNeighborsCellId(cell.first, &cellsId, &cellsValidity);

		for (size_t neighborCellId = 0; neighborCellId < powerOf3<N>::value; ++neighborCellId)
		{
			// Check neighbor's cell validity and use symmetry between pair of cells
			// to only treat neighbors with a larger or equal id.
			if (cellsValidity[neighborCellId] && cellsId[neighborCellId] >= cell.first)
			{
				auto neighborCell = m_activeCells.find(cellsId[neighborCellId]);
				if (neighborCell != m_activeCells.end())
				{
					cell.second.neighbors.insert(cell.second.neighbors.end(),
						neighborCell->second.elements.cbegin(), neighborCell->second.elements.cend());

					// Treat symmetry if the cells are different
					if (cellsId[neighborCellId] != cell.first)
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

	auto const foundCell = m_cellIds.find(element);
	if (foundCell != m_cellIds.cend())
	{
		return m_activeCells[foundCell->second].neighbors;
	}

	return empty;
}

template <typename T, size_t N>
void Grid<T, N>::getNeighborsCellId(size_t cellId,
	Eigen::Matrix<size_t, powerOf3<N>::value, 1>* cellsId,
	Eigen::Matrix<bool, powerOf3<N>::value, 1>* cellsValidity)
{
	// In which cell are we in the dimension-N array ?
	Eigen::Matrix<int, N, 1> cellIdnDOriginal;
	mapping1dToNd(cellId, &cellIdnDOriginal);

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

	cellIdnDOriginal -= Eigen::Matrix<int, N, 1>::Ones();

	Number<int, 3, N> currentNumberNDigitBase3;
	for (size_t i = 0; i < powerOf3<N>::value; ++i)
	{
		mappingNdTo1d(cellIdnDOriginal + currentNumberNDigitBase3, &(*cellsId)[i], &(*cellsValidity)[i]);
		currentNumberNDigitBase3.next();
	}
}

template <typename T, size_t N>
size_t Grid<T, N>::mappingNdTo1d(const Eigen::Matrix<int, N, 1>& nd) const
{
	size_t oned = 0;

	for (size_t i = 0; i < N; ++i)
	{
		oned |= static_cast<size_t>(nd[i]) << m_offsetExponents[i];
	}

	return oned;
}

template <typename T, size_t N>
void Grid<T, N>::mappingNdTo1d(const Eigen::Matrix<int, N, 1>& nd, size_t* oned, bool* valid) const
{
	if ((nd.array() < 0).any() ||
		(nd.template cast<size_t>().array() >= m_numCells.array()).any())
	{
		*valid = false;
	}

	*oned = mappingNdTo1d(nd);
	*valid = true;
}

template <typename T, size_t N>
void Grid<T, N>::mapping1dToNd(size_t oned, Eigen::Matrix<int, N, 1>* nd) const
{
	for (size_t i = 0; i < N; ++i)
	{
		(*nd)[i] = static_cast<int>(oned >> m_offsetExponents[i]);
		oned -= static_cast<size_t>((*nd)[i]) << m_offsetExponents[i];
	}
}

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_GRID_INL_H
