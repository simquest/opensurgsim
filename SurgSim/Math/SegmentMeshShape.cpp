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

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::SegmentMeshShape, SegmentMeshShape);

SegmentMeshShape::SegmentMeshShape()
{
	setRadius(1e-2);
	updateAabbTree();
}

SegmentMeshShape::SegmentMeshShape(const SurgSim::DataStructures::SegmentMeshPlain& mesh, double radius)
	: SurgSim::DataStructures::SegmentMeshPlain(mesh)
{
	setRadius(radius);
	updateAabbTree();
}

int SegmentMeshShape::getType() const
{
	return SHAPE_TYPE_SEGMENTMESH;
}

double SegmentMeshShape::getVolume() const
{
	return std::numeric_limits<double>::quiet_NaN();
}

Vector3d SegmentMeshShape::getCenter() const
{
	return Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

Matrix33d SegmentMeshShape::getSecondMomentOfVolume() const
{
	return Matrix33d::Constant(std::numeric_limits<double>::quiet_NaN());
}

bool SegmentMeshShape::isValid() const
{
	return (m_radius > 1e-5);
}

void SegmentMeshShape::setRadius(double radius)
{
	m_radius = radius;
	double boxHalfDiagonal = m_radius * std::sqrt(3.0);
	m_boxHalfDiagonal = Vector3d(-boxHalfDiagonal, -boxHalfDiagonal, -boxHalfDiagonal);
}

double SegmentMeshShape::getRadius()
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
	if (!SurgSim::DataStructures::SegmentMeshPlain::doLoad(fileName))
	{
		return false;
	}
	return update();
}

const std::shared_ptr<const SurgSim::DataStructures::AabbTree> SegmentMeshShape::getAabbTree() const
{
	return m_aabbTree;
}

void SegmentMeshShape::updateAabbTree()
{
	m_aabbTree = std::make_shared<SurgSim::DataStructures::AabbTree>();

	std::list<DataStructures::AabbTreeData::Item> items;

	auto const& edges = getEdges();

	for (size_t id = 0, count = edges.size(); id < count; ++id)
	{
		if (edges[id].isValid)
		{
			const auto& vertices = getEdgePositions(id);
			Aabbd aabb(SurgSim::Math::makeAabb((vertices[0] + m_boxHalfDiagonal).eval(),
				(vertices[0] - m_boxHalfDiagonal).eval(), (vertices[1] + m_boxHalfDiagonal).eval(),
				(vertices[1] - m_boxHalfDiagonal).eval()));
			items.emplace_back(std::make_pair(std::move(aabb), id));
		}
	}
	m_aabbTree->set(std::move(items));
}

}; // namespace Math
}; // namespace SurgSim
