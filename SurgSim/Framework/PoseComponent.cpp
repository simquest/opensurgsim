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

#include "SurgSim/Framework/PoseComponent.h"

#include "SurgSim/Math/MathConvert.h"

namespace SurgSim
{
namespace Framework
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Framework::PoseComponent, PoseComponent);

PoseComponent::PoseComponent(const std::string& name) : Component(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PoseComponent, SurgSim::Math::UnalignedRigidTransform3d, Pose, getPose, setPose);
}

void PoseComponent::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose = pose;
}

const SurgSim::Math::UnalignedRigidTransform3d& PoseComponent::getPose() const
{
	return m_pose;
}

std::shared_ptr<const PoseComponent> PoseComponent::getPoseComponent() const
{
	return nullptr;
}

std::shared_ptr<PoseComponent> PoseComponent::getPoseComponent()
{
	return nullptr;
}

bool PoseComponent::doInitialize()
{
	return true;
}

bool PoseComponent::doWakeUp()
{
	return true;
}


}; // namespace Framework
}; // namespace SurgSim
