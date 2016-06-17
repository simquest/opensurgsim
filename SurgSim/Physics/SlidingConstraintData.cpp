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

#include "SurgSim/Physics/SlidingConstraintData.h"

namespace SurgSim
{

namespace Physics
{

SlidingConstraintData::SlidingConstraintData() :
	ConstraintData()
{
	m_normals[0].setZero();
	m_normals[1].setZero();
	m_tangent.setZero();
}

SlidingConstraintData::SlidingConstraintData(const SurgSim::Math::Vector3d& point,
											 const SurgSim::Math::Vector3d& direction) :
	ConstraintData()
{
	setSlidingDirection(point, direction);
}

SlidingConstraintData::~SlidingConstraintData()
{
}

void SlidingConstraintData::setSlidingDirection(const SurgSim::Math::Vector3d& point,
												const SurgSim::Math::Vector3d& direction)
{
	Math::Vector3d normal, binormal, tangent;
	normal = direction;
	Math::buildOrthonormalBasis(&normal, &binormal, &tangent);

	m_normals[0] = binormal;
	m_distances[0] = -point.dot(binormal);
	m_normals[1] = tangent;
	m_distances[1] = -point.dot(tangent);
	m_tangent = normal;
	m_distanceTangent = -point.dot(normal);
}

const std::array<Math::Vector3d, 2>& SlidingConstraintData::getNormals() const
{
	return m_normals;
}

const Math::Vector3d& SlidingConstraintData::getTangent() const
{
	return m_tangent;
}

const std::array<double, 2>& SlidingConstraintData::getDistances() const
{
	return m_distances;
}

const double SlidingConstraintData::getDistanceTangent() const
{
	return m_distanceTangent;
}

} // namespace Physics

} // namespace SurgSim
