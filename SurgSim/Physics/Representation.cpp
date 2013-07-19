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

#include <SurgSim/Physics/Representation.h>

#include <SurgSim/Physics/Localization.h>
#include <SurgSim/Physics/Location.h>

#include <SurgSim/Math/MlcpSolution.h>

namespace SurgSim
{
namespace Physics
{

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_gravity(0.0, -9.81, 0.0),
	m_numDof(0),
	m_isGravityEnabled(true),
	m_isActive(true)
{

}

Representation::~Representation()
{

}

void Representation::resetState()
{

}

void Representation::resetParameters()
{

}

void Representation::setIsActive(bool isActive)
{
	m_isActive = isActive;
}

void Representation::setIsGravityEnabled(bool isGravityEnabled)
{
	m_isGravityEnabled = isGravityEnabled;
}

bool Representation::isGravityEnabled() const
{
	return m_isGravityEnabled;
}

void Representation::beforeUpdate(double dt)
{

}

void Representation::update(double dt)
{

}

void Representation::afterUpdate(double dt)
{

}

std::shared_ptr<Localization> Representation::createLocalization(const Location& location)
{
	return nullptr;
}

void Representation::applyDofCorrection(double dt, const Eigen::Block<SurgSim::Math::MlcpSolution::Vector>& block)
{

}

void Representation::setNumDof(unsigned int numDof)
{
	m_numDof = numDof;
}

const SurgSim::Math::Vector3d& Representation::getGravity() const
{
	return m_gravity;
}

}; // Physics
}; // SurgSim
