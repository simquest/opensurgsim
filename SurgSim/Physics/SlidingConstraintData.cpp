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
#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{

namespace Physics
{

SlidingConstraintData::SlidingConstraintData() :
	ConstraintData()
{
	m_normals[0].setZero();
	m_normals[1].setZero();
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
	m_point = point;
	m_slidingDirection = direction;
	Math::buildOrthonormalBasis(&m_slidingDirection, &m_normals[0], &m_normals[1]);
}

const std::array<Math::Vector3d, 2>& SlidingConstraintData::getNormals() const
{
	return m_normals;
}

const Math::RigidTransform3d SlidingConstraintData::getPose()
{
	SurgSim::Math::Matrix33d rotation;
	rotation << m_slidingDirection, m_normals[0], m_normals[1];
	return SurgSim::Math::makeRigidTransform(rotation, m_point);
}

} // namespace Physics

} // namespace SurgSim
