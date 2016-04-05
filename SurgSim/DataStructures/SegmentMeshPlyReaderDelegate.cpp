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

#include "SurgSim/DataStructures/SegmentMeshPlyReaderDelegate.h"

namespace SurgSim
{

namespace DataStructures
{

SegmentMeshPlyReaderDelegate::SegmentMeshPlyReaderDelegate() :
	m_radius(0.0)
{

}

SegmentMeshPlyReaderDelegate::SegmentMeshPlyReaderDelegate(const std::shared_ptr<Math::SegmentMeshShape>& shape) :
	TriangleMeshPlyReaderDelegate<Math::SegmentMeshShape>(shape),
	m_radius(0.0)
{

}

SegmentMeshPlyReaderDelegate::~SegmentMeshPlyReaderDelegate()
{

}

void* SegmentMeshPlyReaderDelegate::beginRadius(const std::string&, size_t value)
{
	SURGSIM_ASSERT(value <= 1) << "Can't process more than 1 radius";
	return &m_radius;
}

void SegmentMeshPlyReaderDelegate::processRadius(const std::string&)
{
	m_shape->setRadius(m_radius);
}

bool SegmentMeshPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	if (m_hasRadius)
	{
		reader->requestElement("radius",
							   std::bind(&SegmentMeshPlyReaderDelegate::beginRadius, this,
										 std::placeholders::_1, std::placeholders::_2),
							   std::bind(&SegmentMeshPlyReaderDelegate::processRadius, this, std::placeholders::_1),
							   nullptr);
		reader->requestScalarProperty("radius", "value", PlyReader::TYPE_DOUBLE, 0);
	}
}

bool SegmentMeshPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = TriangleMeshPlyReaderDelegate<SegmentMeshPlain>::fileIsAcceptable(reader);

	m_hasRadius = reader.hasProperty("radius", "value") && reader.isScalar("radius", "value");

	return result;
}

}
}