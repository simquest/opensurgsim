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

#include <SurgSim/Physics/DeformableRepresentationState.h>

namespace SurgSim
{

namespace Physics
{

DeformableRepresentationState::DeformableRepresentationState()
{
}

DeformableRepresentationState::~DeformableRepresentationState()
{
}

bool DeformableRepresentationState::operator ==(const DeformableRepresentationState& state) const
{
	return m_x == state.m_x && m_v == state.m_v;
}

bool DeformableRepresentationState::operator !=(const DeformableRepresentationState& state) const
{
	return ! ((*this) == state);
}

void DeformableRepresentationState::reset()
{
	m_x.setZero();
	m_v.setZero();
}

void DeformableRepresentationState::allocate(int numDof)
{
	m_x.resize(numDof);
	m_v.resize(numDof);
}

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& DeformableRepresentationState::getPositions()
{
	return m_x;
}

const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& DeformableRepresentationState::getPositions() const
{
	return m_x;
}

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& DeformableRepresentationState::getVelocities()
{
	return m_v;
}

const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& DeformableRepresentationState::getVelocities() const
{
	return m_v;
}

}; // namespace Physics

}; // namespace SurgSim

