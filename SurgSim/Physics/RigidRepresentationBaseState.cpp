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
#include "SurgSim/Physics/RigidRepresentationBaseState.h"

namespace SurgSim
{

namespace Physics
{

RigidRepresentationBaseState::RigidRepresentationBaseState() :
	m_pose(SurgSim::Math::RigidTransform3d::Identity())
{
	addSerializableProperty();
}

RigidRepresentationBaseState::RigidRepresentationBaseState(const RigidRepresentationBaseState& rhs) :
	m_pose(rhs.m_pose)
{
	addSerializableProperty();
}

void RigidRepresentationBaseState::addSerializableProperty()
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationBaseState, SurgSim::Math::RigidTransform3d, Pose,
									  getPose, setPose);
}

RigidRepresentationBaseState& RigidRepresentationBaseState::operator=(const RigidRepresentationBaseState& rhs)
{
	m_pose = rhs.m_pose;
	return *this;
}

RigidRepresentationBaseState::~RigidRepresentationBaseState()
{
}

bool RigidRepresentationBaseState::operator==(const RigidRepresentationBaseState& rhs) const
{
	return m_pose.isApprox(rhs.m_pose);
}

bool RigidRepresentationBaseState::operator!=( const RigidRepresentationBaseState &rhs ) const
{
	return !((*this) == rhs);
}

void RigidRepresentationBaseState::reset()
{
	m_pose.setIdentity();
}

void RigidRepresentationBaseState::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose = pose;
}

const SurgSim::Math::RigidTransform3d& RigidRepresentationBaseState::getPose() const
{
	return m_pose;
}

}; // Physics
}; // SurgSim