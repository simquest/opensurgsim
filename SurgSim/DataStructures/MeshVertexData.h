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

#ifndef SURGSIM_DATA_STRUCTURES_MESH_VERTEX_DATA_H
#define SURGSIM_DATA_STRUCTURES_MESH_VERTEX_DATA_H

#include <SurgSim/Math/Vector.h>

#include <array>
#include <vector>

namespace SurgSim
{

namespace DataStructures
{

/// Abstract base class for data to be stored in a mesh vertex.
/// \sa	MeshVertex
class MeshVertexData
{
public:
	/// Default constructor
	MeshVertexData();
	/// Destructor
	virtual ~MeshVertexData();

	/// Compares the data and returns true if equal, false if not equal.
	bool operator==(const MeshVertexData& data) const;

	/// Compares the data and returns false if equal, true if not equal.
	bool operator!=(const MeshVertexData& data) const;

private:
	/// Internal comparison of data of the same type: returns true if equal, false if not equal.
	/// Override this method to provide custom comparison.
	/// \param	data	Data must be of the same type as that which it is compared against
	virtual bool isEqual(const MeshVertexData& data) const = 0;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_MESH_VERTEX_DATA_H
