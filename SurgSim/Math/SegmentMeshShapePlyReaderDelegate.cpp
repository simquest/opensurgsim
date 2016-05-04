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

#include "SurgSim/Math/SegmentMeshShapePlyReaderDelegate.h"

namespace SurgSim
{

namespace Math
{

SegmentMeshShapePlyReaderDelegate::SegmentMeshShapePlyReaderDelegate() :
	m_radius(0.0)
{

}

SegmentMeshShapePlyReaderDelegate::SegmentMeshShapePlyReaderDelegate(
	const std::shared_ptr<Math::SegmentMeshShape>& shape) :
	TriangleMeshPlyReaderDelegate<Math::SegmentMeshShape>(shape),
	m_hasRadius(false),
	m_radius(0.0),
	m_shape(shape)
{
	SURGSIM_ASSERT(m_shape != nullptr) << "Shape can't be nullptr.";
}

SegmentMeshShapePlyReaderDelegate::~SegmentMeshShapePlyReaderDelegate()
{

}

void* SegmentMeshShapePlyReaderDelegate::beginRadius(const std::string&, size_t value)
{
	SURGSIM_ASSERT(value <= 1) << "Can't process more than 1 radius";
	return &m_radius;
}

void SegmentMeshShapePlyReaderDelegate::processRadius(const std::string&)
{
	m_shape->setRadius(m_radius);
}

bool SegmentMeshShapePlyReaderDelegate::registerDelegate(DataStructures::PlyReader* reader)
{
	bool result = TriangleMeshPlyReaderDelegate<Math::SegmentMeshShape>::registerDelegate(reader);

	if (m_hasRadius)
	{
		reader->requestElement("radius",
							   std::bind(&SegmentMeshShapePlyReaderDelegate::beginRadius, this,
										 std::placeholders::_1, std::placeholders::_2),
							   std::bind(&SegmentMeshShapePlyReaderDelegate::processRadius,
										 this, std::placeholders::_1),
							   nullptr);
		reader->requestScalarProperty("radius", "value", DataStructures::PlyReader::TYPE_DOUBLE, 0);
	}

	return result;
}

bool SegmentMeshShapePlyReaderDelegate::fileIsAcceptable(const DataStructures::PlyReader& reader)
{
	bool result = TriangleMeshPlyReaderDelegate<Math::SegmentMeshShape>::fileIsAcceptable(reader);

	m_hasRadius = reader.hasProperty("radius", "value") && reader.isScalar("radius", "value");

	return result;
}

}
}