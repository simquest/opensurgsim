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
#include "SurgSim/DataStructures/AabbTreeData.h"


namespace SurgSim
{
namespace Math
{

SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::ParticlesShape, ParticlesShape);

ParticlesShape::ParticlesShape(double radius) :
	m_radius(radius)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(ParticlesShape, double, Radius, getRadius, setRadius);
	update();
}

ParticlesShape::ParticlesShape(const ParticlesShape& other) :
	DataStructures::Vertices<DataStructures::EmptyData>(other),
	m_aabbTree(other.m_aabbTree),
	m_radius(other.getRadius()),
	m_center(other.getCenter()),
	m_volume(other.getVolume()),
	m_secondMomentOfVolume(other.getSecondMomentOfVolume())
{
}

int ParticlesShape::getType() const
{
	return SHAPE_TYPE_PARTICLES;
}

double ParticlesShape::getVolume() const
{
	return m_volume;
}

Vector3d ParticlesShape::getCenter() const
{
	return m_center;
}

Matrix33d ParticlesShape::getSecondMomentOfVolume() const
{
	return m_secondMomentOfVolume;
}

bool ParticlesShape::isValid() const
{
	return m_radius >= 0.0;
}

double ParticlesShape::getRadius() const
{
	return m_radius;
}

void ParticlesShape::setRadius(double radius)
{
	m_radius = radius;
}

bool ParticlesShape::doUpdate()
{
	const double numParticles = static_cast<double>(getVertices().size());
	const Vector3d radius = Vector3d::Constant(m_radius);

	std::list<DataStructures::AabbTreeData::Item> items;
	Vector3d totalPosition = Vector3d::Zero();
	Matrix33d totalDisplacementSkewSquared = Matrix33d::Zero();
	size_t id = 0;
	for (auto const& vertex : getVertices())
	{
		totalPosition += vertex.position;

		Matrix33d skewOfDisplacement = makeSkewSymmetricMatrix(vertex.position);
		totalDisplacementSkewSquared += skewOfDisplacement * skewOfDisplacement;

		SurgSim::Math::Aabbd aabb(vertex.position - radius, vertex.position + radius);
		items.emplace_back(std::make_pair(std::move(aabb), id));

		id++;
	}

	m_center = totalPosition / numParticles;

	double sphereVolume = (4.0 / 3.0) * M_PI * m_radius * m_radius * m_radius;
	m_volume =  sphereVolume * numParticles;

	// Parallel Axis Theorem
	m_secondMomentOfVolume = Matrix33d::Identity() * (2.0 / 5.0) * sphereVolume * m_radius * m_radius * numParticles;
	m_secondMomentOfVolume -= sphereVolume * totalDisplacementSkewSquared;

	m_aabbTree = std::make_shared<SurgSim::DataStructures::AabbTree>();
	m_aabbTree->set(std::move(items));

	return true;
}

std::shared_ptr<Shape> ParticlesShape::getTransformed(const RigidTransform3d& pose) const
{
	auto transformed = std::make_shared<ParticlesShape>(*this);
	transformed->transform(pose);
	transformed->update();
	return transformed;
}

const std::shared_ptr<const SurgSim::DataStructures::AabbTree> ParticlesShape::getAabbTree() const
{
	return m_aabbTree;
}

bool ParticlesShape::isTransformable() const
{
	return true;
}

const Math::Aabbd ParticlesShape::getBoundingBox() const
{
	if (m_aabbTree != nullptr)
	{
		return m_aabbTree->getAabb();
	}
	else
	{
		return m_aabb;
	}
}

};
};
