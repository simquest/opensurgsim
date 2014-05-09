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
#include "SurgSim/Physics/RigidRepresentationState.h"

namespace SurgSim
{

namespace Physics
{

RigidRepresentationState::RigidRepresentationState() :
	RigidRepresentationBaseState(),
	m_v(SurgSim::Math::Vector3d::Zero()),
	m_w(SurgSim::Math::Vector3d::Zero())
{
	addSerializableProperty();
}

RigidRepresentationState::RigidRepresentationState(const RigidRepresentationState& rhs) :
	RigidRepresentationBaseState(rhs),
	m_v(rhs.m_v),
	m_w(rhs.m_w)
{
	addSerializableProperty();
}

void RigidRepresentationState::addSerializableProperty()
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationState, SurgSim::Math::Vector3d, LinearVelocity,
									  getLinearVelocity, setLinearVelocity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationState, SurgSim::Math::Vector3d, AngularVelocity,
									  getAngularVelocity, setAngularVelocity);
}

RigidRepresentationState::~RigidRepresentationState()
{
}

bool RigidRepresentationState::operator==(const RigidRepresentationState& rhs) const
{
	return (RigidRepresentationBaseState::operator==(rhs) && m_v == rhs.m_v && m_w == rhs.m_w);
}

bool RigidRepresentationState::operator!=(const RigidRepresentationState& rhs) const
{
	return !((*this) == rhs);
}

void RigidRepresentationState::reset()
{
	RigidRepresentationBaseState::reset();

	m_v.setZero();
	m_w.setZero();
}

const SurgSim::Math::Vector3d& RigidRepresentationState::getLinearVelocity() const
{
	return m_v;
}

const SurgSim::Math::Vector3d& RigidRepresentationState::getAngularVelocity() const
{
	return m_w;
}

void RigidRepresentationState::setLinearVelocity(const SurgSim::Math::Vector3d &v)
{
	m_v = v;
}

void RigidRepresentationState::setAngularVelocity(const SurgSim::Math::Vector3d &w)
{
	m_w = w;
}

}; // Physics
}; // SurgSim