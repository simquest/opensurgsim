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
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/ParticleSystemRepresentation.h"

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
	m_source = checkAndConvert<SurgSim::Particles::ParticleSystemRepresentation>(
				   source, "SurgSim::Particles::ParticleSystemRepresentation");
}

void TransferParticlesToPointCloudBehavior::setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
{
	SURGSIM_ASSERT(nullptr != target) << "'target' can not be nullptr.";
	m_target = checkAndConvert<SurgSim::Graphics::PointCloudRepresentation>(
				   target, "SurgSim::Graphics::PointCloudRepresentation");
}

std::shared_ptr<SurgSim::Particles::ParticleSystemRepresentation>
	TransferParticlesToPointCloudBehavior::getSource() const
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
	auto target = m_target->getVertices();
	size_t nodeId = 0;

	for (auto particle : m_source->getParticleReferences())
	{
		target->setVertexPosition(nodeId, particle.getPosition());
		nodeId++;
	}

	for (; nodeId < m_source->getMaxParticles(); ++nodeId)
	{
		target->setVertexPosition(nodeId, SurgSim::Math::Vector3d::Zero());
	}
}

bool TransferParticlesToPointCloudBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_source != nullptr) << "SetSource must be called prior to initialization";
	SURGSIM_ASSERT(m_target != nullptr) << "SetTarget must be called prior to initialization";

	return true;
}

bool TransferParticlesToPointCloudBehavior::doWakeUp()
{
	auto target = m_target->getVertices();

	if (target->getNumVertices() == 0)
	{
		size_t nodeId = 0;

		for (auto particle : m_source->getParticleReferences())
		{
			SurgSim::Graphics::PointCloud::VertexType vertex(particle.getPosition());
			target->addVertex(vertex);
			nodeId++;
		}

		for (; nodeId < m_source->getMaxParticles(); ++nodeId)
		{
			SurgSim::Graphics::PointCloud::VertexType vertex(SurgSim::Math::Vector3d::Zero());
			target->addVertex(vertex);
		}
	}

	return true;
}

}; //namespace Blocks
}; //namespace SurgSim
