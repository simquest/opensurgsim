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

#ifndef SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_INL_H
#define SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_INL_H

namespace SurgSim
{

namespace Physics
{


inline void VtcRigidRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_currentVtcState.setPose(pose);
}

inline const SurgSim::Math::RigidTransform3d& VtcRigidRepresentation::getPose() const 
{
	return m_finalState.getPose();
}

inline void VtcRigidRepresentation::setInitialParameters(const RigidRepresentationParameters& parameters)
{
	m_initialParameters = parameters;
	m_currentParameters = parameters;

	updateGlobalInertiaMatrices(m_currentState);
}

inline void VtcRigidRepresentation::setCurrentParameters(const RigidRepresentationParameters& parameters)
{
	m_currentParameters = parameters;

	updateGlobalInertiaMatrices(m_currentState);
}

inline void VtcRigidRepresentation::setInitialVtcState(const RigidRepresentationState& state)
{
	m_initialVtcState = state;
	m_currentVtcState = state;
	m_previousVtcState = state;
}

inline void VtcRigidRepresentation::setInitialVtcParameters(const VtcRigidParameters& parameters)
{
	m_initialVtcParameters = parameters;
	m_currentVtcParameters = parameters;
}

inline void VtcRigidRepresentation::setCurrentVtcParameters(const VtcRigidParameters& parameters)
{
	m_currentVtcParameters = parameters;
}

inline const RigidRepresentationState& VtcRigidRepresentation::getInitialVtcState() const
{
	return m_initialVtcState;
}

inline const VtcRigidParameters& VtcRigidRepresentation::getInitialVtcParameters() const
{
	return m_initialVtcParameters;
}

inline const RigidRepresentationState& VtcRigidRepresentation::getCurrentVtcState() const
{
	return m_currentVtcState;
}

inline const RigidRepresentationState& VtcRigidRepresentation::getPreviousVtcState() const
{
	return m_previousVtcState;
}

inline const VtcRigidParameters& VtcRigidRepresentation::getCurrentVtcParameters() const
{
	return m_currentVtcParameters;
}

inline RepresentationType VtcRigidRepresentation::getType() const
{
	return REPRESENTATION_TYPE_VTC_RIGID;
}

inline void VtcRigidRepresentation::resetParameters()
{
	RigidRepresentationBase::resetParameters();

	m_currentParameters = m_initialParameters;

	updateGlobalInertiaMatrices(m_currentState);
}

inline void VtcRigidRepresentation::resetVtcParameters()
{
	m_currentVtcParameters = m_initialVtcParameters;
}

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_INL_H
