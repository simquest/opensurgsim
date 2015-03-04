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

#ifndef SURGSIM_DATASTRUCTURES_LOCATION_H
#define SURGSIM_DATASTRUCTURES_LOCATION_H

#include <vector>

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

struct Location
{
public:
	/// Default constructor
	Location()
	{
	}

	/// Copy constructor
	/// \param other The location to be copied while constructing.
	Location(const Location& other)
		: rigidLocalPosition(other.rigidLocalPosition), octreeNodePath(other.octreeNodePath),
		meshLocalCoordinate(other.meshLocalCoordinate)
	{}

	/// Constructor for rigid local position
	/// \param localPosition The 3D local position to set this location to
	explicit Location(const SurgSim::Math::Vector3d& localPosition)
	{
		rigidLocalPosition.setValue(localPosition);
	}

	/// Constructor for octree node path
	/// \param nodePath The octree node path to set this location to
	explicit Location(const SurgSim::DataStructures::OctreePath& nodePath)
	{
		octreeNodePath.setValue(nodePath);
	}

	/// Constructor for mesh local coordinate
	/// \param localCoordinate The mesh local coordinate
	explicit Location(const SurgSim::DataStructures::IndexedLocalCoordinate& localCoordinate)
	{
		meshLocalCoordinate.setValue(localCoordinate);
	}

	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d> rigidLocalPosition;
	SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::OctreePath> octreeNodePath;
	SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::IndexedLocalCoordinate> meshLocalCoordinate;
};

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_LOCATION_H
