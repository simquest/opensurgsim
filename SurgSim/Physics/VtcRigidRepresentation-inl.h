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
