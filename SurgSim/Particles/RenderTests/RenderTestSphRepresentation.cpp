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

///\file RenderTestSphRepresentation.cpp render test for SphRepresentation

#include <memory>

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/EmitterRepresentation.h"
#include "SurgSim/Particles/RandomSpherePointGenerator.h"
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/RenderTests/RenderTest.h"
#include "SurgSim/Particles/SphRepresentation.h"
//#include "SurgSim/Particles/UnitTests/MockObject.h"

using SurgSim::Framework::Behavior;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Particles::RenderTests;
using SurgSim::Particles::SphRepresentation;

namespace SurgSim
{
namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(TransferParticlesToPointCloudBehavior);

/// Behavior to copy positions of a ParticlesRepresentation to a PointCloud.
class TransferParticlesToPointCloudBehavior : public Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit TransferParticlesToPointCloudBehavior(const std::string& name) :
		Behavior(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferParticlesToPointCloudBehavior,
			std::shared_ptr<SurgSim::Framework::Component>, Source, getSource, setSource);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(TransferParticlesToPointCloudBehavior,
			std::shared_ptr<SurgSim::Framework::Component>, Target, getTarget, setTarget);
	}

	SURGSIM_CLASSNAME(SurgSim::Blocks::TransferParticlesToPointCloudBehavior);

	/// Set the representation from which the positions are from
	/// \param source The physics representation
	void setSource(const std::shared_ptr<SurgSim::Framework::Component>& source)
	{
		using SurgSim::Framework::checkAndConvert;

		SURGSIM_ASSERT(nullptr != source) << "'source' can not be nullptr.";
		m_source = checkAndConvert<SurgSim::Particles::ParticleSystemRepresentation>(
			source, "SurgSim::Particles::ParticleSystemRepresentation");
	}

	/// Set the point cloud representation which will receive the positions
	/// \param target The Graphics PointCloud representation
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component>& target)
	{
		using SurgSim::Framework::checkAndConvert;

		SURGSIM_ASSERT(nullptr != target) << "'target' can not be nullptr.";
		m_target = checkAndConvert<SurgSim::Graphics::PointCloudRepresentation>(
			target, "SurgSim::Graphics::PointCloudRepresentation");
	}

	/// Get the ParticleSystem representation which sends the positions
	/// \return The particles system representation which produces positions.
	std::shared_ptr<SurgSim::Particles::ParticleSystemRepresentation> getSource() const
	{
		return m_source;
	}

	/// Get the point cloud representation which receives the positions
	/// \return The Graphics PointCloud representation which receives positions.
	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation> getTarget() const
	{
		return m_target;
	}

	virtual void update(double dt) override
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

private:
	virtual bool doInitialize() override
	{
		return true;
	}

	virtual bool doWakeUp() override
	{
		auto target = m_target->getVertices();

		if (target->getNumVertices() == 0)
		{
			for (size_t nodeId = 0; nodeId < m_source->getMaxParticles(); ++nodeId)
			{
				SurgSim::Graphics::PointCloud::VertexType vertex(SurgSim::Math::Vector3d::Zero());
				target->addVertex(vertex);
			}
		}
		return true;
	}

	/// The ParticleSystem from which the positions come from.
	std::shared_ptr<SurgSim::Particles::ParticleSystemRepresentation> m_source;

	/// The Graphics PointCloud Representation to which the vertices' positions are set.
	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation> m_target;
};

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::TransferParticlesToPointCloudBehavior,
				 TransferParticlesToPointCloudBehavior);

};  // namespace Blocks
};  // namespace SurgSim






namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createParticleSystem(const std::string& name,
		SurgSim::Math::Vector4d color)
{
	// Create the scene element that will contain all the necessary object (phx, gfx, behaviors...)
	std::shared_ptr<SurgSim::Framework::BasicSceneElement> sceneElement =
		std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Create the particle physics object and add it to the element
	std::shared_ptr<SurgSim::Particles::SphRepresentation> particlesRepresentation =
		std::make_shared<SurgSim::Particles::SphRepresentation>(name + " Particles");

	// c.f. "Lagrangian Fluid Dynamics Using Smoothed Particle Hydrodynamics", Micky Kelager, January 9th 2006.
	// for input data to simulate water.
	particlesRepresentation->setMaxParticles(5000);
	particlesRepresentation->setMassPerParticle(0.02);
	particlesRepresentation->setDensityReference(998.29);
	particlesRepresentation->setGasStiffness(3.0);
	particlesRepresentation->setGravity(SurgSim::Math::Vector3d(0.0, -9.81, 0.0));
	particlesRepresentation->setKernelSupport(0.0457);
	particlesRepresentation->setSurfaceTensionCoefficient(0.0728);
	particlesRepresentation->setViscosity(3.5);

	SphRepresentation::PlaneConstraint planeConstraint;
	planeConstraint.stiffness = 50000.0;
	planeConstraint.damping = 100.0;
	planeConstraint.planeEquation = SurgSim::Math::Vector4d(0.0, 1.0, 0.0, 0.0);
	particlesRepresentation->addPlaneConstraint(planeConstraint);
	planeConstraint.planeEquation.segment<3>(0) = Vector3d(-1,0.9,0).normalized();
	particlesRepresentation->addPlaneConstraint(planeConstraint);
	planeConstraint.planeEquation.segment<3>(0) = Vector3d(+1,0.9,0).normalized();
	particlesRepresentation->addPlaneConstraint(planeConstraint);
	planeConstraint.planeEquation.segment<3>(0) = Vector3d(0,0.9,+1).normalized();
	particlesRepresentation->addPlaneConstraint(planeConstraint);
	planeConstraint.planeEquation.segment<3>(0) = Vector3d(0,0.9,-1).normalized();
	particlesRepresentation->addPlaneConstraint(planeConstraint);
	sceneElement->addComponent(particlesRepresentation);

	std::shared_ptr<SurgSim::Particles::EmitterRepresentation> sphereEmitter;
	sphereEmitter = std::make_shared<SurgSim::Particles::EmitterRepresentation>("sphereEmitter");
	sphereEmitter->setLocalPose(SurgSim::Math::makeRigidTranslation(SurgSim::Math::Vector3d(0.5, 1.0, 0.5)));
	sphereEmitter->setTarget(particlesRepresentation);
	sphereEmitter->setShape(std::make_shared<SurgSim::Math::SphereShape>(0.1));
	sphereEmitter->setMode(SurgSim::Particles::EMIT_MODE_SURFACE);
	sphereEmitter->setRate(1000.0); /// 500 particles per second
	sphereEmitter->setLifetimeRange(std::make_pair(30000, 600000));
	sphereEmitter->setVelocityRange(std::make_pair(SurgSim::Math::Vector3d::Zero(), SurgSim::Math::Vector3d::Zero()));
	sceneElement->addComponent(sphereEmitter);

	// Create the particle graphics object and add it to the element
	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation =
				std::make_shared<OsgPointCloudRepresentation>("Graphics object");
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);
	sceneElement->addComponent(graphicsRepresentation);

	// Create the transfer behavior from particle physics to graphics and add it to the element
	auto particlesToGraphics =
		std::make_shared<SurgSim::Blocks::TransferParticlesToPointCloudBehavior>("Particles to Graphics");
	particlesToGraphics->setSource(particlesRepresentation);
	particlesToGraphics->setTarget(graphicsRepresentation);
	sceneElement->addComponent(particlesToGraphics);

	return sceneElement;
}

}; // namespace anonymous

TEST_F(RenderTests, SphRenderTest)
{
	scene->addSceneElement(createParticleSystem("Particles", Vector4d::Ones()));

	particlesManager->setRate(500.0);
	runTest(Vector3d(0.0, 0.0, 8.5), Vector3d::Zero(), 30000.0);
}
