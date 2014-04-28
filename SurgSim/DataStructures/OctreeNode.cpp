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
#include <cmath>
#include <fstream>

namespace SurgSim
{

namespace DataStructures
{

std::shared_ptr<OctreeNode<EmptyData>> loadOctree(const std::string& fileName)
{
	std::ifstream octreeData(fileName, std::ios::in);
	SURGSIM_ASSERT(octreeData) << "Could not open file (" << fileName << ")" << std::endl;

	SurgSim::Math::Vector3d spacing,boundsMin, boundsMax;
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

};  // namespace DataStructures
};  // namespace SurgSim

