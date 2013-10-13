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

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_INL_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_INL_H

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{

namespace Physics
{

template <class M, class D, class K>
DeformableRepresentation<M,D,K>::DeformableRepresentation(const std::string& name) :
Representation(name)
{
	m_initialPose.setIdentity();
	m_identityPose.setIdentity();
}

template <class M, class D, class K>
DeformableRepresentation<M,D,K>::~DeformableRepresentation()
{
}

template <class M, class D, class K>
void DeformableRepresentation<M,D,K>::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
}

template <class M, class D, class K>
const SurgSim::Math::RigidTransform3d& DeformableRepresentation<M,D,K>::getInitialPose() const
{
	return m_initialPose;
}

template <class M, class D, class K>
void DeformableRepresentation<M,D,K>::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_ASSERT(false) << "setPose has been called on a physics DeformableRepresentation";
}

template <class M, class D, class K>
const SurgSim::Math::RigidTransform3d& DeformableRepresentation<M,D,K>::getPose() const
{
	return m_identityPose;
}

template <class M, class D, class K>
void DeformableRepresentation<M,D,K>::resetState()
{
	Representation::resetState();

	*m_currentState  = *m_initialState;
	*m_previousState = *m_initialState;
	*m_finalState    = *m_initialState;
}

template <class M, class D, class K>
void DeformableRepresentation<M,D,K>::setInitialState(std::shared_ptr<DeformableRepresentationState> initialState)
{
	// This initializes and allocates the m_initialState data member
	m_initialState = initialState;
	transformState(m_initialState, m_initialPose);

	m_previousState = std::make_shared<DeformableRepresentationState>(*initialState);
	m_currentState = std::make_shared<DeformableRepresentationState>(*initialState);
	m_newState = std::make_shared<DeformableRepresentationState>(*initialState);
	m_finalState = std::make_shared<DeformableRepresentationState>(*initialState);

	// Set the representation number of degree of freedom
	setNumDof(initialState->getNumDof());
}

template <class M, class D, class K>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K>::getInitialState() const
{
	return m_initialState;
}

template <class M, class D, class K>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K>::getCurrentState() const
{
	return m_currentState;
}

template <class M, class D, class K>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K>::getPreviousState() const
{
	return m_previousState;
}

template <class M, class D, class K>
const std::shared_ptr<DeformableRepresentationState> DeformableRepresentation<M,D,K>::getFinalState() const
{
	return m_finalState;
}

}; // namespace Physics

}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_DEFORMABLEREPRESENTATION_INL_H
