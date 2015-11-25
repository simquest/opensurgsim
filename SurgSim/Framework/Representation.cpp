// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Representation.h"

#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/MathConvert.h"


namespace SurgSim
{
namespace Framework
{

Representation::Representation(const std::string& m_name) :
	Component(m_name), m_localPose(Math::RigidTransform3d::Identity())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, Math::RigidTransform3d, LocalPose, getLocalPose, setLocalPose);
	SURGSIM_ADD_RO_PROPERTY(Representation, Math::RigidTransform3d, Pose, getPose);
}

Representation::~Representation()
{
}

bool Representation::doInitialize()
{
	return true;
}

bool Representation::doWakeUp()
{
	return true;
}

void Representation::setLocalPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_localPose = pose;
}

SurgSim::Math::RigidTransform3d Representation::getPose() const
{
	auto element = getSceneElement();
	if (element != nullptr)
	{
		return element->getPose() * getLocalPose();
	}
	else
	{
		return getLocalPose();
	}
}

SurgSim::Math::RigidTransform3d Representation::getLocalPose() const
{
	return m_localPose;
}

}; // namespace Framework
}; // namespace SurgSim
