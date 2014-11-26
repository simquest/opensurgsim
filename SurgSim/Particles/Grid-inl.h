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

#ifndef SURGSIM_PARTICLES_GRID_INL_H
#define SURGSIM_PARTICLES_GRID_INL_H

namespace SurgSim
{
namespace Particles
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
/// \tparam B The base in which the number is expressed (must be less within [2..9])
/// \tparam N The number of digit for this number
template <typename T, size_t B, size_t N>
class Number : public Eigen::Matrix<T, N, 1>
{
public:
	Number()
	{
		this->setZero();
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
Grid<T, N>::Grid() : m_size(0.0), m_numCells(0u)
{
	m_numCellsPerDimension.setZero();
	m_powerOf2CellsPerDimension.setZero();

	m_offsetPerDimension.setZero();
	m_offsetPowerOf2PerDimension.setZero();
}

template <typename T, size_t N>
Grid<T, N>::~Grid()
{
}

template <typename T, size_t N>
void Grid<T, N>::init(double size, const Eigen::Matrix<size_t, N, 1>& powerOf2CellsPerDimension)
{
	// Check that the size of the requested grid fit the current architecture.
	size_t maximumCellsPowerOf2 = sizeof(size_t) * 8;
	size_t requestedCellsPozerOf2 = powerOf2CellsPerDimension.sum();
	SURGSIM_ASSERT(requestedCellsPozerOf2 <= maximumCellsPowerOf2) << "The requested grid dimension (2^"
		<< requestedCellsPozerOf2 << " cells) is too large for the "
		<< maximumCellsPowerOf2 << " bit architecture";

	m_size = size;
	m_powerOf2CellsPerDimension = powerOf2CellsPerDimension;
	m_numCells = 1u;
	for (size_t i = 0; i < N; ++i)
	{
		m_numCellsPerDimension[i] = (static_cast<size_t>(1u) << powerOf2CellsPerDimension[i]);
		m_numCells *= m_numCellsPerDimension[i];
		m_aabb.min()[i] = -static_cast<double>(m_numCellsPerDimension[i]) * m_size * 0.5;
		m_aabb.max()[i] = static_cast<double>(m_numCellsPerDimension[i]) * m_size * 0.5;
	}

	// Cell indexing goes as follow:
	// Example: a cell in 3d with the indices (i, j, k) will have a 1d index of
	// i * m_offsetPerDimension[0] + j * m_offsetPerDimension[1] + k * m_offsetPerDimension[2]
	if (N >= 1)
	{
		m_offsetPerDimension[N - 1] = 1;
		m_offsetPowerOf2PerDimension[N - 1] = 0;
	}
	for (int i = static_cast<int>(N) - 2; i >= 0; i--)
	{
		m_offsetPerDimension[i] = m_offsetPerDimension[i + 1] * m_numCellsPerDimension[i + 1];
		m_offsetPowerOf2PerDimension[i] = m_offsetPowerOf2PerDimension[i + 1] + m_powerOf2CellsPerDimension[i + 1];
	}
}

template <typename T, size_t N>
void Grid<T, N>::reset()
{
	// Clear the mapping element -> cellId
	m_mapElementToCellId.clear();

	// Clear the active cells
	m_activeCells.clear();

	// Clear the neighbors' lists
	m_mapElementNeighbors.clear();
}

template <typename T, size_t N>
template <class Derived>
void Grid<T, N>::addElementAt(const Eigen::MatrixBase<Derived>& position, const T element)
{
	if (!(position.array() >= m_aabb.min().array() && position.array() <= m_aabb.max().array()).all())
	{
		return;
	}

	// Find the n-D cell id from the location
	Eigen::Matrix<int, N, 1> cellIdnD;
	// Example in 3D:
	// cell (i,j,k) has a min/max of
	// min = (size * (-numCellPerDim[0] / 2 + i)   max = (size * (-numCellPerDim[0] / 2 + i + 1)
	//       (size * (-numCellPerDim[1] / 2 + i)         (size * (-numCellPerDim[1] / 2 + i + 1)
	//       (size * (-numCellPerDim[2] / 2 + i)         (size * (-numCellPerDim[2] / 2 + i + 1)
	for (size_t i = 0; i < N; ++i)
	{
		double cellIdForThisDimension = position[i] / m_size + static_cast<double>(m_numCellsPerDimension[i] >> 1);
		cellIdnD[i] = static_cast<int>(floor(cellIdForThisDimension));
	}

	// Find the 1D cell id from the n-D cell id
	size_t cellId1D = cellId_ndTo1d(cellIdnD);

	// Add the cell id to the list of active cell if not present already
	// and add the element into this cell.
	m_activeCells[cellId1D].push_back(element);

	// Add this element in the map [element -> cellID]
	m_mapElementToCellId[element] = cellId1D;
}

template <typename T, size_t N>
void Grid<T, N>::computeNeighborsMap()
{
	// All currently active particles are stored in m_mapElementToCellId
	for (auto element : m_mapElementToCellId)
	{
		getNeighborsOfElement(element.first, &m_mapElementNeighbors[element.first]);
	}
}

template <typename T, size_t N>
const std::unordered_map<T, std::vector<T>>& Grid<T, N>::getNeighborsMap() const
{
	return m_mapElementNeighbors;
}

template <typename T, size_t N>
void Grid<T, N>::getNeighborsOfElement(const T& element, std::vector<T>* result)
{
	Eigen::Matrix<bool, powerOf3<N>::value, 1> neighborCellsValidity;
	Eigen::Matrix<size_t, powerOf3<N>::value, 1> neighborCellsId;
	size_t thisCellId = m_mapElementToCellId[element];

	getNeighborsCellId(thisCellId, &neighborCellsId, &neighborCellsValidity);

	result->clear();

	// Look in the neighbor cells for elements
	for (size_t index = 0; index < powerOf3<N>::value; ++index)
	{
		// Does this neighboring cell exist in the given n-d grid ? Handles border cells...
		if (neighborCellsValidity[index])
		{
			// Does this cell is active ? (i.e. contains anything useful ?)
			auto neighborCellId = neighborCellsId[index];
			if(m_activeCells.find(neighborCellId) != m_activeCells.end())
			{
				result->insert(result->end(), m_activeCells[neighborCellId].cbegin(),
					m_activeCells[neighborCellId].cend());
			}
		}
	}
}

template <typename T, size_t N>
void Grid<T, N>::getNeighborsCellId(size_t cellId,
	Eigen::Matrix<size_t, powerOf3<N>::value, 1>* cellsId,
	Eigen::Matrix<bool, powerOf3<N>::value, 1>* cellsValidity)
{
	// In which cell are we in the n-d array ?
	Eigen::Matrix<int, N, 1> cellIdnDOriginal;
	cellId_1dTond(cellId, &cellIdnDOriginal);

	// Now build up all the neighbors cell around this n-d cell
	// It corresponds to all possible permutation in n-d of the indices
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
		cellId_ndTo1d(cellIdnDOriginal + currentNumberNDigitBase3, &(*cellsId)[i], &(*cellsValidity)[i]);
		currentNumberNDigitBase3.next();
	}
}

template <typename T, size_t N>
size_t Grid<T, N>::cellId_ndTo1d(const Eigen::Matrix<int, N, 1>& cellIdnD) const
{
	size_t cellId1D = 0;

	for (size_t i = 0; i < N; ++i)
	{
		cellId1D |= static_cast<size_t>(cellIdnD[i]) << m_offsetPowerOf2PerDimension[i];
	}

	return cellId1D;
}

template <typename T, size_t N>
void Grid<T, N>::cellId_ndTo1d(const Eigen::Matrix<int, N, 1>& cellIdnD, size_t* cellId1D, bool* valid) const
{
	if ((cellIdnD.array() < 0).any() ||
		(cellIdnD.template cast<size_t>().array() >= m_numCellsPerDimension.array()).any())
	{
		*valid = false;
	}

	*cellId1D = cellId_ndTo1d(cellIdnD);
	*valid = true;
}

template <typename T, size_t N>
void Grid<T, N>::cellId_1dTond(size_t cellId1D, Eigen::Matrix<int, N, 1>* cellIdnD) const
{
	for (size_t i = 0; i < N; ++i)
	{
		(*cellIdnD)[i] = static_cast<int>(cellId1D >> m_offsetPowerOf2PerDimension[i]);
		cellId1D -= static_cast<size_t>((*cellIdnD)[i]) << m_offsetPowerOf2PerDimension[i];
	}
}

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_GRID_INL_H
