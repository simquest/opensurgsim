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

#include <boost/thread/lock_guard.hpp>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace Math
{

Shape::Shape() :
	m_pose(RigidTransform3d::Identity())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Shape, RigidTransform3d, Pose, getPose, setPose);
	m_aabb.setEmpty();
}

Shape::Shape(const RigidTransform3d& pose) :
	m_pose(pose)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Shape, RigidTransform3d, Pose, getPose, setPose);
	m_aabb.setEmpty();
}

Shape::~Shape()
{
}

Vector3d Shape::getCenter() const
{
	return Vector3d::Zero();
}

/// Get class name
std::string Shape::getClassName() const
{
	SURGSIM_LOG_WARNING(Framework::Logger::getDefaultLogger()) <<
			"getClassName() called on Math::Shape base class, this is wrong" <<
			" in almost all cases, this means there is a class that does not have getClassName() defined.";
	return "SurgSim::Math::Shape";
}

Aabbd Shape::getBoundingBox() const
{
	boost::lock_guard<boost::mutex> lock(m_aabbMutex);
	return m_aabb;
}

void Shape::setPose(const RigidTransform3d& pose)
{
	if (!m_pose.isApprox(pose))
	{
		boost::lock_guard<boost::mutex> lock(m_aabbMutex);
		m_pose = pose;
		updateAabb();
	}
}

const RigidTransform3d& Shape::getPose() const
{
	return m_pose;
}

} // namespace Math
} // namespace SurgSim
