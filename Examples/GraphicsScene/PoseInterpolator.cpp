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

#include <Examples/GraphicsScene/PoseInterpolator.h>

#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Graphics/Representation.h>

#include <memory>

using SurgSim::Math::RigidTransform3d;

PoseInterpolator::PoseInterpolator(const std::string& name) :
	Behavior(name),
	m_optionalFrom(RigidTransform3d::Identity()),
	m_to(RigidTransform3d::Identity()),
	m_duration(1.0),
	m_currentTime(0.0)
{

}

void PoseInterpolator::setFrom(const SurgSim::Math::RigidTransform3d transform)
{
	m_optionalFrom.setValue(transform);
}

void PoseInterpolator::setTo(const SurgSim::Math::RigidTransform3d transform)
{
	m_to = transform;
}

void PoseInterpolator::setTarget(std::shared_ptr<SurgSim::Graphics::Representation> target)
{
	m_target = target;
}

void PoseInterpolator::setDuration(double t)
{
	m_duration = t;
}

bool PoseInterpolator::doInitialize()
{
	return true;
}

bool PoseInterpolator::doWakeUp()
{
	bool result = false;
	if (m_target != nullptr)
	{
		m_from = (m_optionalFrom.hasValue()) ? m_optionalFrom.getValue() : m_target->getPose();
		result = true;
	}
	return result;
}

void PoseInterpolator::update(double dt)
{
	bool result = true;
	m_currentTime += dt;

	m_target->setPose(SurgSim::Math::interpolate(m_from, m_to, m_currentTime/m_duration));

	if (m_currentTime > m_duration)
	{
		if (isLoop())
		{
			m_currentTime = 0.0;
		}
		else if (isPingPong())
		{
			m_currentTime = 0.0;
			std::swap(m_from, m_to);
		}
		else
		{
			getSceneElement()->removeComponent(getName());
		}
	}
}
