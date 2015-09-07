// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_SEGMENTEMPTYDATA_H
#define SURGSIM_DATASTRUCTURES_SEGMENTEMPTYDATA_H

namespace SurgSim
{
namespace DataStructures
{
/// SegmentEmptyData class
/// This class is to ensure that the static TriangleMesh::m_className in SegmentMesh does not clash with
/// static TriangleMesh<VertexData, EdgeData, EmptyData>.
class SegmentEmptyData
{
public:

	/// Comparison operator
	/// \param data The data to compare it to.
	/// \return true for all cases.
	bool operator==(const SegmentEmptyData& data) const
	{
		return true;
	}
};

}
}
#endif //SURGSIM_DATASTRUCTURES_SEGMENTEMPTYDATA_H
