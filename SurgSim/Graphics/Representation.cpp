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

#include <SurgSim/Graphics/Representation.h>

#include <SurgSim/Graphics/Material.h>

using SurgSim::Graphics::Material;
using SurgSim::Graphics::Representation;

Representation::Representation(const std::string& name) : SurgSim::Framework::Representation(name),
	m_initialPose(SurgSim::Math::RigidTransform3d::Identity())
{
}

void Representation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_initialPose = pose;
	setPose(m_initialPose);
}

bool Representation::setMaterial(std::shared_ptr<Material> material)
{
	m_material = material;
	return true;
}

void Representation::clearMaterial()
{
	m_material = nullptr;
}
