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

#include <array>
#include <iterator>
#include <cmath>
#include <fstream>

#include <boost/container/static_vector.hpp>


namespace
{

typedef std::pair<size_t, SurgSim::DataStructures::Symbol> TransitionCode;

using SurgSim::DataStructures::SYMBOL_HALT;
using SurgSim::DataStructures::SYMBOL_LEFT;
using SurgSim::DataStructures::SYMBOL_RIGHT;
using SurgSim::DataStructures::SYMBOL_FRONT;
using SurgSim::DataStructures::SYMBOL_BACK;
using SurgSim::DataStructures::SYMBOL_UP;
using SurgSim::DataStructures::SYMBOL_DOWN;

static TransitionCode transitionTable[6][8] =
{
	// Transition codes for 'Down'
	std::make_pair(2, SYMBOL_DOWN), std::make_pair(3, SYMBOL_DOWN),
	std::make_pair(0, SYMBOL_HALT), std::make_pair(1, SYMBOL_HALT),
	std::make_pair(6, SYMBOL_DOWN), std::make_pair(7, SYMBOL_DOWN),
	std::make_pair(4, SYMBOL_HALT), std::make_pair(5, SYMBOL_HALT),

	// Transition Codes for 'Up'
	std::make_pair(2, SYMBOL_HALT), std::make_pair(3, SYMBOL_HALT),
	std::make_pair(0, SYMBOL_UP), std::make_pair(1, SYMBOL_UP),
	std::make_pair(6, SYMBOL_HALT), std::make_pair(7, SYMBOL_HALT),
	std::make_pair(4, SYMBOL_UP), std::make_pair(5, SYMBOL_UP),

	// Transition Codes for 'Right'
	std::make_pair(1, SYMBOL_HALT), std::make_pair(0, SYMBOL_RIGHT),
	std::make_pair(3, SYMBOL_HALT), std::make_pair(2, SYMBOL_RIGHT),
	std::make_pair(5, SYMBOL_HALT), std::make_pair(4, SYMBOL_RIGHT),
	std::make_pair(7, SYMBOL_HALT), std::make_pair(6, SYMBOL_RIGHT),

	// Transition Codes for 'Left'
	std::make_pair(1, SYMBOL_LEFT), std::make_pair(0, SYMBOL_HALT),
	std::make_pair(3, SYMBOL_LEFT), std::make_pair(2, SYMBOL_HALT),
	std::make_pair(5, SYMBOL_LEFT), std::make_pair(4, SYMBOL_HALT),
	std::make_pair(7, SYMBOL_LEFT), std::make_pair(6, SYMBOL_HALT),

	// Transition Codes for 'Back'
	std::make_pair(4, SYMBOL_BACK), std::make_pair(5, SYMBOL_BACK),
	std::make_pair(6, SYMBOL_BACK), std::make_pair(7, SYMBOL_BACK),
	std::make_pair(0, SYMBOL_HALT), std::make_pair(1, SYMBOL_HALT),
	std::make_pair(2, SYMBOL_HALT), std::make_pair(3, SYMBOL_HALT),

	// Transition Codes for 'Front'
	std::make_pair(4, SYMBOL_HALT), std::make_pair(5, SYMBOL_HALT),
	std::make_pair(6, SYMBOL_HALT), std::make_pair(7, SYMBOL_HALT),
	std::make_pair(0, SYMBOL_FRONT), std::make_pair(1, SYMBOL_FRONT),
	std::make_pair(2, SYMBOL_FRONT), std::make_pair(3, SYMBOL_FRONT),
};

static const std::array<std::array<int, 3>, 6> FaceNeighbors =
{
	SYMBOL_DOWN, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_UP, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_RIGHT, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_LEFT, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_BACK, SYMBOL_HALT, SYMBOL_HALT,
	SYMBOL_FRONT, SYMBOL_HALT, SYMBOL_HALT,
};

static const std::array<std::array<int, 3>, 12> EdgeNeighbors =
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

static const std::array<std::array<int, 3>, 8> VertexNeighbors =
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

OctreePath getNeighbor(const OctreePath& source, const std::array<int, 3>& direction)
{
	// Early Exit
	if (source.size() == 0 || direction[0] == SYMBOL_HALT)
	{
		return OctreePath();
	}

	boost::container::static_vector<int, 3> currentDirection;
	boost::container::static_vector<int, 3> newDirection;

	for (size_t i = 0; i < direction.size() && direction[i] != SYMBOL_HALT; ++i)
	{
		currentDirection.push_back(direction[i]);
	}

	OctreePath result(source);

	bool didHalt = false;

	for (ptrdiff_t pathIndex = source.size() - 1; pathIndex >= 0; --pathIndex)
	{
		newDirection.clear();
		size_t currentNodeId = source[pathIndex];
		std::cout << "Current Node : " << currentNodeId << std::endl;
		std::cout << "Direction  : " << direction[0] << ", " << direction[1] << ", " << direction[2] << std::endl;
		for (size_t directionIndex = 0; directionIndex < currentDirection.size(); ++directionIndex)
		{
			TransitionCode code = transitionTable[currentDirection[directionIndex]][currentNodeId];
			currentNodeId = code.first;
			std::cout << "TransitionCode: " << code.first << ", " << code.second << std::endl;
			if (code.second != SYMBOL_HALT)
			{
				newDirection.push_back(code.second);
			}
		}

		currentDirection = newDirection;
		result[pathIndex] = currentNodeId;

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

	std::cout << "Result: ";
	for (auto i : result)
	{
		std::cout << i << ", ";
	}
	std::cout << std::endl;
	return std::move(result);
}

std::vector<OctreePath> getNeighbors(const OctreePath& source, Neighborhood type)
{
	std::vector<OctreePath> result;
	auto f = [&source, &result](const std::array<int, 3>& direction)
	{
		auto n = getNeighbor(source, direction);
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

