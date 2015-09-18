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

#include "SurgSim/Math/SegmentMeshShape.h"

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Geometry.h"


namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::SegmentMeshShape, SegmentMeshShape);

SegmentMeshShape::SegmentMeshShape()
	: m_radius(0.0)
{
	updateAabbTree();
}

int SegmentMeshShape::getType() const
{
	return SHAPE_TYPE_SEGMENTMESH;
}

double SegmentMeshShape::getVolume() const
{
	SURGSIM_ASSERT("SegmentMeshShape: Volume calculation not implemented.");
	return std::numeric_limits<double>::quiet_NaN();
}

Vector3d SegmentMeshShape::getCenter() const
{
	SURGSIM_ASSERT("SegmentMeshShape: Center calculation not implemented.");
	return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

Matrix33d SegmentMeshShape::getSecondMomentOfVolume() const
{
	SURGSIM_ASSERT("SegmentMeshShape: Second Moment Of Volume calculation not implemented.");
	return Matrix33d::Constant(std::numeric_limits<double>::quiet_NaN());
}

bool SegmentMeshShape::isValid() const
{
	// If the radius is less than DistanceEpsilon, the collision detection (which assumes the segments are capsules),
	/// will not work correctly.
	return (m_radius >= Geometry::DistanceEpsilon);
}

void SegmentMeshShape::setRadius(double radius)
{
	m_radius = radius;
	m_segmentEndBoundingBoxHalfExtent = Vector3d(m_radius, m_radius, m_radius);
}

double SegmentMeshShape::getRadius() const
{
	return m_radius;
}

bool SegmentMeshShape::doUpdate()
{
	updateAabbTree();
	return true;
}

bool SegmentMeshShape::doLoad(const std::string& fileName)
{
	return DataStructures::SegmentMeshPlain::doLoad(fileName) && update();
}

std::shared_ptr<const DataStructures::AabbTree> SegmentMeshShape::getAabbTree() const
{
	return m_aabbTree;
}

void SegmentMeshShape::updateAabbTree()
{
	m_aabbTree = std::make_shared<DataStructures::AabbTree>();

	std::list<DataStructures::AabbTreeData::Item> items;

	auto const& edges = getEdges();

	for (size_t id = 0, count = edges.size(); id < count; ++id)
	{
		if (edges[id].isValid)
		{
			const auto& vertices = getEdgePositions(id);
			Aabbd aabb((vertices[0] - m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[0] + m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[1] - m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[1] + m_segmentEndBoundingBoxHalfExtent).eval());
			items.emplace_back(std::make_pair(std::move(aabb), id));
		}
	}
	m_aabbTree->set(std::move(items));
}

}; // namespace Math
}; // namespace SurgSim
