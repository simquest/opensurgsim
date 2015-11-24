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

#include "SurgSim/Blocks/PoseInterpolator.h"

#include "SurgSim/Framework/SceneElement.h"

#include <memory>

using SurgSim::Math::RigidTransform3d;
namespace SurgSim
{
namespace Blocks
{

PoseInterpolator::PoseInterpolator(const std::string& name) :
	Behavior(name),
	m_startingPose(RigidTransform3d::Identity()),
	m_endingPose(RigidTransform3d::Identity()),
	m_duration(1.0),
	m_currentTime(0.0)
{

}

void PoseInterpolator::setStartingPose(const SurgSim::Math::RigidTransform3d& transform)
{
	if (!isInitialized())
	{
		m_optionalStartPose.setValue(transform);
	}
}

void PoseInterpolator::setEndingPose(const SurgSim::Math::RigidTransform3d& transform)
{
	if (!isInitialized())
	{
		m_endingPose = transform;
	}
}

void PoseInterpolator::setTarget(std::shared_ptr<SurgSim::Framework::SceneElement> target)
{
	if (!isInitialized())
	{
		m_target = target;
	}
}

void PoseInterpolator::setDuration(double t)
{
	if (!isInitialized())
	{
		m_duration = t;
	}
}

double PoseInterpolator::getDuration() const
{
	return m_duration;
}

bool PoseInterpolator::doInitialize()
{
	return true;
}

bool PoseInterpolator::doWakeUp()
{
	bool result = false;
	if (m_target == nullptr)
	{
		m_target = getSceneElement();
	}
	if (m_target != nullptr)
	{
		m_startingPose = (m_optionalStartPose.hasValue()) ? m_optionalStartPose.getValue() : m_target->getPose();
		result = true;
	}
	return result;
}

void PoseInterpolator::update(double dt)
{
	m_currentTime += dt;

	if (m_currentTime >= m_duration)
	{
		if (isLoop())
		{
			m_currentTime = m_currentTime - m_duration;
		}
		else if (isPingPong())
		{
			m_currentTime = m_currentTime - m_duration;
			std::swap(m_endingPose, m_startingPose);
		}
		else
		{
			m_currentTime = m_duration;
			auto element = getSceneElement();
			if (element != nullptr)
			{
				element->removeComponent(getName());
			}
		}
	}

	m_target->setPose(SurgSim::Math::interpolate(m_startingPose, m_endingPose, m_currentTime/m_duration));
}

void PoseInterpolator::setLoop(bool val)
{
	m_loop = val;
	if (m_loop)
	{
		m_pingpong = false;
	}
}

bool PoseInterpolator::isLoop() const
{
	return m_loop;
}

void PoseInterpolator::setPingPong(bool val)
{
	m_pingpong = val;
	if (m_pingpong)
	{
		m_loop = false;
	}
}

bool PoseInterpolator::isPingPong() const
{
	return m_pingpong;
}

}; // Blocks
}; // Surgsim
