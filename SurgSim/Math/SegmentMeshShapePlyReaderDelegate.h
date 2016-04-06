// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_SEGMENTMESHPLYREADERDELEGATE_H
#define SURGSIM_DATASTRUCTURES_SEGMENTMESHPLYREADERDELEGATE_H

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/SegmentMesh.h"
#include "SurgSim/Math/SegmentMeshShape.h"


namespace SurgSim
{

namespace Math
{

/// Implementation of ply reader for segment meshes, enable to read the radius from the ply file
class SegmentMeshShapePlyReaderDelegate : public DataStructures::TriangleMeshPlyReaderDelegate<SegmentMeshShape>
{
public:
	/// Constructor
	SegmentMeshShapePlyReaderDelegate();

	explicit SegmentMeshShapePlyReaderDelegate(const std::shared_ptr<Math::SegmentMeshShape>& shape);

	/// Destructor
	~SegmentMeshShapePlyReaderDelegate();

	void* beginRadius(const std::string& element, size_t value);

	void processRadius(const std::string& element);

	bool registerDelegate(DataStructures::PlyReader* reader) override;

	bool fileIsAcceptable(const DataStructures::PlyReader& reader) override;

private:
	bool m_hasRadius;
	double m_radius;

	std::shared_ptr<Math::SegmentMeshShape> m_shape;
};

}
}

#endif
