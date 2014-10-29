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

#include "SurgSim/DataStructures/OctreeNode.h"

#include "SurgSim/Framework/Assert.h"

#include <array>
#include <cmath>
#include <fstream>

#include <boost/container/static_vector.hpp>



namespace
{

using SurgSim::DataStructures::SYMBOL_HALT;
using SurgSim::DataStructures::SYMBOL_LEFT;
using SurgSim::DataStructures::SYMBOL_RIGHT;
using SurgSim::DataStructures::SYMBOL_FRONT;
using SurgSim::DataStructures::SYMBOL_BACK;
using SurgSim::DataStructures::SYMBOL_UP;
using SurgSim::DataStructures::SYMBOL_DOWN;
using SurgSim::DataStructures::Symbol;

struct TransitionCode
{
	size_t nodeId;
	Symbol symbol;
};

/// State machine codes, paired by new Value, new Directions value
static TransitionCode transitionTable[6][8] =
{
	// Transition codes for 'Down'
	{
		{2u, SYMBOL_DOWN}, {3u, SYMBOL_DOWN},
		{0u, SYMBOL_HALT}, {1u, SYMBOL_HALT},
		{6u, SYMBOL_DOWN}, {7u, SYMBOL_DOWN},
		{4u, SYMBOL_HALT}, {5u, SYMBOL_HALT}
	},

	// Transition Codes for 'Up'
	{
		{2u, SYMBOL_HALT}, {3u, SYMBOL_HALT},
		{0u, SYMBOL_UP}, {1u, SYMBOL_UP},
		{6u, SYMBOL_HALT}, {7u, SYMBOL_HALT},
		{4u, SYMBOL_UP}, {5u, SYMBOL_UP}
	},

	// Transition Codes for 'Right'
	{
		{1u, SYMBOL_HALT}, {0u, SYMBOL_RIGHT},
		{3u, SYMBOL_HALT}, {2u, SYMBOL_RIGHT},
		{5u, SYMBOL_HALT}, {4u, SYMBOL_RIGHT},
		{7u, SYMBOL_HALT}, {6u, SYMBOL_RIGHT}
	},

	// Transition Codes for 'Left'
	{
		{1u, SYMBOL_LEFT}, {0u, SYMBOL_HALT},
		{3u, SYMBOL_LEFT}, {2u, SYMBOL_HALT},
		{5u, SYMBOL_LEFT}, {4u, SYMBOL_HALT},
		{7u, SYMBOL_LEFT}, {6u, SYMBOL_HALT}
	},

	// Transition Codes for 'Back'
	{
		{4u, SYMBOL_BACK}, {5u, SYMBOL_BACK},
		{6u, SYMBOL_BACK}, {7u, SYMBOL_BACK},
		{0u, SYMBOL_HALT}, {1u, SYMBOL_HALT},
		{2u, SYMBOL_HALT}, {3u, SYMBOL_HALT}
	},

	// Transition Codes for 'Front'
	{
		{4u, SYMBOL_HALT}, {5u, SYMBOL_HALT},
		{6u, SYMBOL_HALT}, {7u, SYMBOL_HALT},
		{0u, SYMBOL_FRONT}, {1u, SYMBOL_FRONT},
		{2u, SYMBOL_FRONT}, {3u, SYMBOL_FRONT}
	}
};

static const std::array<std::array<Symbol, 3>, 6> FaceNeighbors =
{
	SYMBOL_DOWN, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_UP, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_RIGHT, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_LEFT, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_BACK, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_FRONT, SYMBOL_HALT, SYMBOL_HALT,
};

static const std::array<std::array<Symbol, 3>, 12> EdgeNeighbors =
{
	SYMBOL_DOWN, SYMBOL_RIGHT, SYMBOL_HALT,
	SYMBOL_DOWN, SYMBOL_LEFT, SYMBOL_HALT,
	SYMBOL_DOWN, SYMBOL_FRONT, SYMBOL_HALT,
	SYMBOL_DOWN, SYMBOL_BACK, SYMBOL_HALT,
	SYMBOL_UP, SYMBOL_RIGHT, SYMBOL_HALT,
	SYMBOL_UP, SYMBOL_LEFT, SYMBOL_HALT,
	SYMBOL_UP, SYMBOL_FRONT, SYMBOL_HALT,
	SYMBOL_UP, SYMBOL_BACK, SYMBOL_HALT,
	SYMBOL_BACK, SYMBOL_RIGHT, SYMBOL_HALT,
	SYMBOL_BACK, SYMBOL_LEFT, SYMBOL_HALT,
	SYMBOL_FRONT, SYMBOL_RIGHT, SYMBOL_HALT,
	SYMBOL_FRONT, SYMBOL_LEFT, SYMBOL_HALT,
};

static const std::array<std::array<Symbol, 3>, 8> VertexNeighbors =
{
	SYMBOL_DOWN, SYMBOL_RIGHT, SYMBOL_FRONT,
	SYMBOL_DOWN, SYMBOL_RIGHT, SYMBOL_BACK,
	SYMBOL_DOWN, SYMBOL_LEFT, SYMBOL_FRONT,
	SYMBOL_DOWN, SYMBOL_LEFT, SYMBOL_BACK,
	SYMBOL_UP, SYMBOL_RIGHT, SYMBOL_FRONT,
	SYMBOL_UP, SYMBOL_RIGHT, SYMBOL_BACK,
	SYMBOL_UP, SYMBOL_LEFT, SYMBOL_FRONT,
	SYMBOL_UP, SYMBOL_LEFT, SYMBOL_BACK,
};

}




namespace SurgSim
{

namespace DataStructures
{

SURGSIM_REGISTER(SurgSim::Framework::Asset,
				 SurgSim::DataStructures::OctreeNode<EmptyData>,
				 OctreeNodeEmptyData);

// Predefine classname of OctreeNode of EmptyData
template<>
std::string OctreeNode<EmptyData>::m_className = "SurgSim::DataStructures::OctreeNode<EmptyData>";

std::shared_ptr<OctreeNode<EmptyData>> loadOctree(const std::string& fileName)
{
	std::ifstream octreeData(fileName, std::ios::in);
	SURGSIM_ASSERT(octreeData) << "Could not open file (" << fileName << ")" << std::endl;

	SurgSim::Math::Vector3d spacing, boundsMin, boundsMax;
	std::array<int, 3> dimensions;
	octreeData >> dimensions[0] >> dimensions[1] >> dimensions[2];
	octreeData >> spacing[0] >> spacing[1] >> spacing[2];
	octreeData >> boundsMin[0] >> boundsMax[0] >> boundsMin[1] >> boundsMax[1] >> boundsMin[2] >> boundsMax[2];

	int maxDimension = dimensions[0];
	maxDimension = maxDimension >= dimensions[1] ?
				   (maxDimension >= dimensions[2] ? maxDimension : dimensions[2]) :
				   (dimensions[1] >= dimensions[2] ? dimensions[1] : dimensions[2]);

	int numLevels = static_cast<int>(std::ceil(std::log(maxDimension) / std::log(2.0)));
	SurgSim::Math::Vector3d octreeDimensions = SurgSim::Math::Vector3d::Ones() * std::pow(2.0, numLevels);

	typedef OctreeNode<SurgSim::DataStructures::EmptyData> OctreeNodeType;
	OctreeNodeType::AxisAlignedBoundingBox boundingBox;
	boundingBox.min() = boundsMin;
	boundingBox.max() = boundsMin.array() + octreeDimensions.array() * spacing.array();
	std::shared_ptr<OctreeNodeType> octree = std::make_shared<OctreeNodeType>(boundingBox);

	SurgSim::Math::Vector3d position;
	while (octreeData >> position[0] >> position[1] >> position[2])
	{
		octree->addData(position, SurgSim::DataStructures::EmptyData(), numLevels);
	}
	return octree;
}

SurgSim::DataStructures::OctreePath getNeighbor(const OctreePath& origin, const std::array<Symbol, 3>& direction)
{
	// Early Exit
	if (origin.size() == 0 || direction[0] == SYMBOL_HALT)
	{
		return OctreePath();
	}

	boost::container::static_vector<int, 3> currentDirection;
	boost::container::static_vector<int, 3> newDirection;

	for (size_t i = 0; i < direction.size() && direction[i] != SYMBOL_HALT; ++i)
	{
		currentDirection.push_back(direction[i]);
	}

	OctreePath result(origin);

	bool didHalt = false;

	// For each level iterate over all directions and remember the new node value, and any directions
	// if there were any new directions (transitionCode.second != SYMBOL_HALT) then do the same for the
	// preceding level. If there are no new directions then we are done and the superior level are unchanged
	// If after the topmost level was dealt with there are still directions to work with, the neighbor is outside
	// of the octree, an empty array is returned in that case.

	for (ptrdiff_t pathIndex = origin.size() - 1; pathIndex >= 0; --pathIndex)
	{
		newDirection.clear();
		size_t currentNodeId = origin[pathIndex];

		for (size_t directionIndex = 0; directionIndex < currentDirection.size(); ++directionIndex)
		{
			TransitionCode code = transitionTable[currentDirection[directionIndex]][currentNodeId];
			currentNodeId = code.nodeId;
			if (code.symbol != SYMBOL_HALT)
			{
				newDirection.push_back(code.symbol);
			}
		}

		currentDirection = newDirection;
		result[pathIndex] = currentNodeId;

		// If now other new direction was found then we are done and can break of the algorithm
		if (currentDirection.empty())
		{
			didHalt = true;
			break;
		}
	}

	// If the last symbol was not a Halt, then the node is outside of the tree
	if (!didHalt)
	{
		result.clear();
	}

	return std::move(result);
}

std::vector<OctreePath> getNeighbors(const OctreePath& origin, int type)
{
	std::vector<OctreePath> result;
	auto f = [&origin, &result](const std::array<Symbol, 3>& direction)
	{
		auto n = getNeighbor(origin, direction);
		if (! n.empty())
		{
			result.push_back(std::move(n));
		}
	};

	size_t size = (((NEIGHBORHOOD_FACE & type) == 0) ? 0 : 6) +
				  (((NEIGHBORHOOD_EDGE & type) == 0) ? 0 : 12) +
				  (((NEIGHBORHOOD_VERTEX & type) == 0) ? 0 : 8);

	result.reserve(size);

	if ((NEIGHBORHOOD_FACE & type) != 0)
	{
		std::for_each(FaceNeighbors.begin(), FaceNeighbors.end(), f);
	}
	if ((NEIGHBORHOOD_EDGE & type) != 0)
	{
		std::for_each(EdgeNeighbors.begin(), EdgeNeighbors.end(), f);
	}
	if ((NEIGHBORHOOD_VERTEX & type) != 0)
	{
		std::for_each(VertexNeighbors.begin(), VertexNeighbors.end(), f);
	}
	return std::move(result);
}

};  // namespace DataStructures
};  // namespace SurgSim

