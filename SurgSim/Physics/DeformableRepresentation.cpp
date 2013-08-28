// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Base class for all deformable representations (abstract class)

#include <SurgSim/Physics/DeformableRepresentation.h>

namespace SurgSim
{

namespace Physics
{

DeformableRepresentation::DeformableRepresentation(const std::string& name) :
Representation(name)
{
	m_initialPose.setIdentity();
}

DeformableRepresentation::~DeformableRepresentation()
{
}

void DeformableRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
}

const SurgSim::Math::RigidTransform3d& DeformableRepresentation::getInitialPose() const
{
	return m_initialPose;
}

void DeformableRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_ASSERT(false) << "setPose has been called on a physics DeformableRepresentation";
}

const SurgSim::Math::RigidTransform3d& DeformableRepresentation::getPose() const
{
	static const SurgSim::Math::RigidTransform3d staticLocalId = SurgSim::Math::RigidTransform3d::Identity();
	return staticLocalId;
}

void DeformableRepresentation::resetState()
{
	Representation::resetState();

	m_currentState  = m_initialState;
	m_previousState = m_initialState;
	m_finalState    = m_initialState;
}

const DeformableRepresentationState& DeformableRepresentation::getInitialState() const
{
	return m_initialState;
}

const DeformableRepresentationState& DeformableRepresentation::getCurrentState() const
{
	return m_currentState;
}

const DeformableRepresentationState& DeformableRepresentation::getPreviousState() const
{
	return m_previousState;
}

const DeformableRepresentationState& DeformableRepresentation::getFinalState() const
{
	return m_finalState;
}

}; // namespace Physics

}; // namespace SurgSim
