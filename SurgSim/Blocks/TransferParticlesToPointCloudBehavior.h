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

#ifndef SURGSIM_BLOCKS_TRANSFERPARTICLESTOPOINTCLOUDBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPARTICLESTOPOINTCLOUDBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Macros.h"

namespace SurgSim
{

namespace Framework
{
class Component;
}

namespace Graphics
{
class PointCloudRepresentation;
}

namespace Particles
{
class ParticleSystemRepresentation;
}

namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(TransferParticlesToPointCloudBehavior);

/// Behavior to copy positions of a ParticleSystemRepresentation to a PointCloud.
class TransferParticlesToPointCloudBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit TransferParticlesToPointCloudBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::TransferParticlesToPointCloudBehavior);

	/// Set the representation from which the positions are from
	/// \param source The particles representation
	void setSource(const std::shared_ptr<SurgSim::Framework::Component>& source);

	/// Set the point cloud representation which will receive the positions
	/// \param target The Graphics PointCloud representation
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target);

	/// Get the particles representation which sends the positions
	/// \return The Physics representation which produces positions.
	std::shared_ptr<SurgSim::Particles::ParticleSystemRepresentation> getSource() const;

	/// Get the point cloud representation which receives the positions
	/// \return The Graphics PointCloud representation which receives positions.
	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation> getTarget() const;

	virtual void update(double dt) override;

private:
	virtual bool doInitialize() override;
	virtual bool doWakeUp() override;

	/// The ParticleSystemRepresentation from which the positions come.
	std::shared_ptr<SurgSim::Particles::ParticleSystemRepresentation> m_source;

	/// The Graphics PointCloud Representation to which the vertices's positions are set.
	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation> m_target;
};

};  // namespace Blocks
};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_TRANSFERPARTICLESTOPOINTCLOUDBEHAVIOR_H
