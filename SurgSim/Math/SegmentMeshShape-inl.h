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

#ifndef SURGSIM_MATH_SEGMENTMESHSHAPE_INL_H
#define SURGSIM_MATH_SEGMENTMESHSHAPE_INL_H

namespace SurgSim
{
namespace Math
{

template <class VertexData, class EdgeData>
SegmentMeshShape::SegmentMeshShape(
	const SurgSim::DataStructures::SegmentMesh<VertexData, EdgeData>& mesh,
	double radius) : SurgSim::DataStructures::SegmentMeshPlain(mesh),
	m_initialVertices(mesh)
{
	setRadius(radius);
	updateAabbTree();
}

}; // namespace Math
}; // namespace SurgSim

#endif // SURGSIM_MATH_SEGMENTMESHSHAPE_INL_H
