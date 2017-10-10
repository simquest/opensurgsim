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
#include "SurgSim/Math/SegmentMeshShapePlyReaderDelegate.h"


namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::SegmentMeshShape, SegmentMeshShape);

SegmentMeshShape::SegmentMeshShape()
{
	setRadius(0.001);
	buildAabbTree();
}

SegmentMeshShape::SegmentMeshShape(const SegmentMeshShape& other) :
	DataStructures::SegmentMeshPlain(other)
{
	setRadius(other.m_radius);
	setInitialVertices(other.getInitialVertices());
	buildAabbTree();
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
	return (m_radius >= Geometry::DistanceEpsilon &&
			m_segmentEndBoundingBoxHalfExtent.isApprox(Vector3d(m_radius, m_radius, m_radius)));
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
	buildAabbTree();
	return true;
}

bool SegmentMeshShape::doLoad(const std::string& fileName)
{
	DataStructures::PlyReader reader(fileName);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
				<< "'" << fileName << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<SegmentMeshShapePlyReaderDelegate>(
						std::dynamic_pointer_cast<SegmentMeshShape>(this->shared_from_this()));
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
				<< "The input file '" << fileName << "' does not have the property required by segment mesh.";
		return false;
	}

	return true;
}

std::shared_ptr<const DataStructures::AabbTree> SegmentMeshShape::getAabbTree() const
{
	return m_aabbTree;
}

std::shared_ptr<Shape> SegmentMeshShape::getTransformed(const RigidTransform3d& pose) const
{
	auto transformed = std::make_shared<SegmentMeshShape>(*this);
	transformed->transform(pose);
	transformed->update();
	return transformed;
}

void SegmentMeshShape::buildAabbTree()
{
	m_aabbTree = std::make_shared<DataStructures::AabbTree>();

	SurgSim::DataStructures::AabbTreeData::ItemList items;

	auto const& edges = getEdges();

	for (size_t id = 0; id < edges.size(); ++id)
	{
		if (edges[id].isValid)
		{
			const auto& vertices = getEdgePositions(id);
			Aabbd aabb((vertices[0] - m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[0] + m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[1] - m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[1] + m_segmentEndBoundingBoxHalfExtent).eval());
			items.emplace_back(aabb, id);
		}
	}
	m_aabbTree->set(std::move(items));
	m_aabb = m_aabbTree->getAabb();
}

void SegmentMeshShape::updateAabbTree()
{
	auto const& edges = getEdges();
	m_aabbCache.resize(edges.size());

	for (size_t id = 0; id < edges.size(); ++id)
	{
		if (edges[id].isValid)
		{
			const auto& vertices = getEdgePositions(id);
			Aabbd aabb((vertices[0] - m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[0] + m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[1] - m_segmentEndBoundingBoxHalfExtent).eval());
			aabb.extend((vertices[1] + m_segmentEndBoundingBoxHalfExtent).eval());
			m_aabbCache[id] = aabb;
		}
	}
	m_aabbTree->updateBounds(m_aabbCache);
	m_aabb = m_aabbTree->getAabb();
}

void SegmentMeshShape::setPose(const RigidTransform3d& pose)
{
	auto& vertices = getVertices();
	const size_t numVertices = vertices.size();
	const auto& initialVertices = m_initialVertices.getVertices();
	SURGSIM_ASSERT(numVertices == initialVertices.size()) <<
			"SegmentMeshShape cannot update vertices' positions because of mismatched size: currently " <<
			numVertices << " vertices, vs initially " << initialVertices.size() << " vertices.";
	for (size_t i = 0; i < numVertices; ++i)
	{
		vertices[i].position = pose * initialVertices[i].position;
	}
}

}; // namespace Math
}; // namespace SurgSim
