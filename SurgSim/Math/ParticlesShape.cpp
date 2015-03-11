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

#include "SurgSim/Math/ParticlesShape.h"

#include "SurgSim/DataStructures/AabbTree.h"


namespace SurgSim
{
namespace Math
{

ParticlesShape::ParticlesShape() :
	m_radius(0.01)
{
}

int ParticlesShape::getType() const
{
	return SHAPE_TYPE_PARTICLES;
}

double ParticlesShape::getVolume() const
{
	SURGSIM_FAILURE() << "ParticlesShape::getVolume not implemented";
	return 0.0;
}

Vector3d ParticlesShape::getCenter() const
{
	return Vector3d::Zero();
}

Matrix33d ParticlesShape::getSecondMomentOfVolume() const
{
	SURGSIM_FAILURE() << "ParticlesShape::getSecondMomentOfVolume not implemented";
	return Matrix33d::Zero();
}

bool ParticlesShape::isValid() const
{
	return true;
}

double ParticlesShape::getRadius() const
{
	return m_radius;
}

void ParticlesShape::setRadius(double radius)
{
	m_radius = radius;
}

void ParticlesShape::doUpdate()
{
	auto aabbTree = std::make_shared<SurgSim::DataStructures::AabbTree>();
	auto const& vertices = getVertices();
	const Vector3d radius = Vector3d::Constant(m_radius);

	for (size_t id = 0; id < vertices.size(); ++id)
	{
		const Vector3d& position = vertices[id].position;
		aabbTree->add(SurgSim::Math::Aabbd(position - radius, position + radius), id);
	}

	m_aabbTree = aabbTree;
}

const std::shared_ptr<const SurgSim::DataStructures::AabbTree> ParticlesShape::getAabbTree() const
{
	return m_aabbTree;
}

};
};
