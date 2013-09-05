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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATION_INL_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATION_INL_H

namespace SurgSim
{

namespace Physics
{

inline void SurgSim::Physics::RigidRepresentation::setInitialParameters(const RigidRepresentationParameters& parameters)
{
	m_initialParameters = parameters;
	m_currentParameters = parameters;

	updateGlobalInertiaMatrices(m_currentState);
}


inline void SurgSim::Physics::RigidRepresentation::setCurrentParameters(const RigidRepresentationParameters& parameters)
{
	m_currentParameters = parameters;
	updateGlobalInertiaMatrices(m_currentState);
}


inline void SurgSim::Physics::RigidRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
}


inline void SurgSim::Physics::RigidRepresentation::resetParameters()
{
	Representation::resetParameters();
	m_currentParameters = m_initialParameters;

	updateGlobalInertiaMatrices(m_currentState);
}


inline const Eigen::Matrix<double, 6,6, Eigen::DontAlign | Eigen::RowMajor>&
	SurgSim::Physics::RigidRepresentation::getComplianceMatrix() const
{
	return m_C;
}

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATION_INL_H
