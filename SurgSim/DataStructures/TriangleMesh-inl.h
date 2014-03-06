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
//

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H

namespace SurgSim
{
namespace DataStructures
{

template <class VertexDataSource, class EdgeDataSource, class TriangleDataSource>
TriangleMesh::TriangleMesh(
	const std::shared_ptr<TriangleMeshBase<VertexDataSource, EdgeDataSource, TriangleDataSource>> mesh) :
	TriangleMeshBase<EmptyData, EmptyData, TriangleData>(*mesh)
{
	calculateNormals();
}


}; // DataStructures
}; // SurgSim

#endif // SURGSIM_DATASTRUCTURES_TRIANGLEMESH_INL_H