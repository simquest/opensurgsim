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

#ifndef SURGSIM_DATASTRUCTURES_SEGMENTMESH_H
#define SURGSIM_DATASTRUCTURES_SEGMENTMESH_H

#include "SurgSim/DataStructures/TriangleMesh.h"

namespace SurgSim
{
namespace DataStructures
{

/// Class to hold the type of a SegmentMesh.
template <class VertexData, class EdgeData>
struct SegmentMesh
{
	typedef TriangleMesh<VertexData, EdgeData, EmptyData> Type;
};

typedef SegmentMesh<EmptyData, EmptyData>::Type SegmentMeshPlain;

}  // namespace DataStructures
}  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_SEGMENTMESH_H
