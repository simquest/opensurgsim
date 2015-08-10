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

#include "SurgSim/Blocks/TransferParticlesToPointCloudBehavior.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/DataStructures/Vertex.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Particles/Representation.h"

using SurgSim::Framework::checkAndConvert;

namespace SurgSim
{

namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::TransferParticlesToPointCloudBehavior,
				 TransferParticlesToPointCloudBehavior);

TransferParticlesToPointCloudBehavior::TransferParticlesToPointCloudBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferParticlesToPointCloudBehavior,
									  std::shared_ptr<SurgSim::Framework::Component>, Source, getSource, setSource);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferParticlesToPointCloudBehavior,
									  std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget, setTarget);
}

void TransferParticlesToPointCloudBehavior::setSource(const std::shared_ptr<SurgSim::Framework::Component>& source)
{
	SURGSIM_ASSERT(nullptr != source) << "'source' can not be nullptr.";
	m_source = checkAndConvert<SurgSim::Particles::Representation>(
				   source, "SurgSim::Particles::Representation");
}

void TransferParticlesToPointCloudBehavior::setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << "'target' can not be nullptr.";
	m_target = checkAndConvert<SurgSim::Graphics::PointCloudRepresentation>(
				   target, "SurgSim::Graphics::PointCloudRepresentation");
}

std::shared_ptr<SurgSim::Particles::Representation> TransferParticlesToPointCloudBehavior::getSource() const
{
	return m_source;
}

std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation>
	TransferParticlesToPointCloudBehavior::getTarget() const
{
	return m_target;
}

void TransferParticlesToPointCloudBehavior::update(double dt)
{
	*m_target->getVertices() = m_source->getParticles();
}

bool TransferParticlesToPointCloudBehavior::doInitialize()
{
	return true;
}

bool TransferParticlesToPointCloudBehavior::doWakeUp()
{
	if (m_source == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
				getName() + "' must have a source.";
		return false;
	}
	if (m_target == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
				getName() + "' must have a target.";
		return false;
	}

	return true;
}

}; //namespace Blocks
}; //namespace SurgSim
