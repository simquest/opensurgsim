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

#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Physics/RigidState.h"

namespace SurgSim
{

namespace Physics
{

RigidState::RigidState() :
	m_v(SurgSim::Math::Vector3d::Zero()),
	m_w(SurgSim::Math::Vector3d::Zero()),
	m_pose(SurgSim::Math::RigidTransform3d::Identity())
{
	addSerializableProperty();
}

RigidState::RigidState(const RigidState& rhs) :
	m_v(rhs.m_v),
	m_w(rhs.m_w),
	m_pose(rhs.m_pose)
{
	addSerializableProperty();
}

void RigidState::addSerializableProperty()
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidState, SurgSim::Math::Vector3d, LinearVelocity,
									  getLinearVelocity, setLinearVelocity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidState, SurgSim::Math::Vector3d, AngularVelocity,
									  getAngularVelocity, setAngularVelocity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidState, SurgSim::Math::RigidTransform3d, Pose,
									  getPose, setPose);
}


RigidState& RigidState::operator=(const RigidState& rhs)
{
	m_v = rhs.m_v;
	m_w = rhs.m_w;
	m_pose = rhs.m_pose;

	return *this;
}


RigidState::~RigidState()
{
}

bool RigidState::operator==(const RigidState& rhs) const
{
	return (m_pose.isApprox(rhs.m_pose) && m_v == rhs.m_v && m_w == rhs.m_w);
}

bool RigidState::operator!=(const RigidState& rhs) const
{
	return !((*this) == rhs);
}

void RigidState::reset()
{
	m_v.setZero();
	m_w.setZero();
	m_pose.setIdentity();
}

const SurgSim::Math::Vector3d& RigidState::getLinearVelocity() const
{
	return m_v;
}

const SurgSim::Math::Vector3d& RigidState::getAngularVelocity() const
{
	return m_w;
}

void RigidState::setLinearVelocity(const SurgSim::Math::Vector3d &v)
{
	m_v = v;
}

void RigidState::setAngularVelocity(const SurgSim::Math::Vector3d &w)
{
	m_w = w;
}

void RigidState::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose = pose;
}

const SurgSim::Math::RigidTransform3d& RigidState::getPose() const
{
	return m_pose;
}

}; // Physics
}; // SurgSim