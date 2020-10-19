// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Particles/SphRepresentation.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Particles
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Particles::SphRepresentation, SphRepresentation);

SphRepresentation::SphRepresentation(const std::string& name) :
	Representation(name),
	m_mass(0.0),
	m_densityReference(0.0),
	m_gasStiffness(0.0),
	m_surfaceTension(0.0),
	m_stiffness(0.0),
	m_damping(0.0),
	m_friction(0.0),
	m_gravity(Math::Vector3d(0.0, -9.81, 0.0)),
	m_viscosity(0.0),
	m_h(0.0),
	m_hSquared(std::numeric_limits<double>::signaling_NaN()),
	m_kernelPoly6(std::numeric_limits<double>::signaling_NaN()),
	m_kernelPoly6Gradient(std::numeric_limits<double>::signaling_NaN()),
	m_kernelSpikyGradient(std::numeric_limits<double>::signaling_NaN()),
	m_kernelViscosityLaplacian(std::numeric_limits<double>::signaling_NaN()),
	m_kernelPoly6Laplacian(std::numeric_limits<double>::signaling_NaN())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, MassPerParticle, getMassPerParticle,
			setMassPerParticle);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, Density, getDensity, setDensity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, GasStiffness, getGasStiffness, setGasStiffness);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, SurfaceTension, getSurfaceTension, setSurfaceTension);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, Viscosity, getViscosity, setViscosity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, Stiffness, getStiffness, setStiffness);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, Damping, getDamping, setDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, Friction, getFriction, setFriction);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double, KernelSupport, getKernelSupport, setKernelSupport);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, Math::Vector3d, Gravity, getGravity, setGravity);
}

SphRepresentation::~SphRepresentation()
{
}

void SphRepresentation::setMassPerParticle(double particleMass)
{
	SURGSIM_ASSERT(particleMass > 0.0) <<
		"The mass per particle needs to be a valid positive non null value." << particleMass << " was provided.";
	m_mass = particleMass;
}

double SphRepresentation::getMassPerParticle() const
{
	return m_mass;
}

void SphRepresentation::setDensity(double density)
{
	SURGSIM_ASSERT(density > 0.0) <<
		"The density needs to be a valid positive non null value." << density << " was provided.";
	m_densityReference = density;
}

double SphRepresentation::getDensity() const
{
	return m_densityReference;
}

void SphRepresentation::setGasStiffness(double stiffness)
{
	SURGSIM_ASSERT(stiffness > 0.0) <<
		"The gas stiffness needs to be a valid positive non null value." << stiffness << " was provided.";
	m_gasStiffness = stiffness;
}

double SphRepresentation::getGasStiffness() const
{
	return m_gasStiffness;
}

void SphRepresentation::setSurfaceTension(double surfaceTension)
{
	SURGSIM_ASSERT(surfaceTension >= 0.0) <<
		"The surface tension needs to be a valid positive or null value." <<
		surfaceTension << " was provided.";
	m_surfaceTension = surfaceTension;
}

double SphRepresentation::getSurfaceTension() const
{
	return m_surfaceTension;
}

void SphRepresentation::setGravity(const SurgSim::Math::Vector3d& gravity)
{
	m_gravity = gravity;
}

SurgSim::Math::Vector3d SphRepresentation::getGravity() const
{
	return m_gravity;
}

void SphRepresentation::setViscosity(double viscosity)
{
	SURGSIM_ASSERT(viscosity >= 0.0) <<
		"The viscosity needs to be a valid positive or null value." << viscosity << " was provided.";
	m_viscosity = viscosity;
}

double SphRepresentation::getViscosity() const
{
	return m_viscosity;
}

void SphRepresentation::setKernelSupport(double support)
{
	SURGSIM_ASSERT(support > 0.0) <<
		"The kernel support needs to be a valid positive non-null value." << support << " was provided.";

	m_h = support;
	m_hSquared = m_h * m_h;
	m_kernelPoly6 = 315.0 / (64.0 * M_PI * std::pow(m_h, 9));
	m_kernelPoly6Gradient = -945.0 / (32.0 * M_PI * std::pow(m_h, 9));
	m_kernelSpikyGradient = -45.0 / (M_PI * std::pow(m_h, 6));
	m_kernelViscosityLaplacian = 45.0 / (M_PI * std::pow(m_h, 5));
	m_kernelPoly6Laplacian = 945.0 / (8.0 * M_PI * std::pow(m_h, 9));
}

double SphRepresentation::getKernelSupport() const
{
	return m_h;
}

void SphRepresentation::setStiffness(double stiffness)
{
	m_stiffness = stiffness;
}

double SphRepresentation::getStiffness() const
{
	return m_stiffness;
}

void SphRepresentation::setDamping(double damping)
{
	m_damping = damping;
}

double SphRepresentation::getDamping() const
{
	return m_damping;
}

void SphRepresentation::setFriction(double friction)
{
	m_friction = friction;
}

double SphRepresentation::getFriction() const
{
	return m_friction;
}

bool SphRepresentation::doInitialize()
{
	if (!Representation::doInitialize())
	{
		return false;
	}

	SURGSIM_ASSERT(m_mass > 0.0) <<
		"The mass per particle needs to be set prior to adding the component in the SceneElement";
	SURGSIM_ASSERT(m_densityReference > 0.0) <<
		"The reference density needs to be set prior to adding the component in the SceneElement";
	SURGSIM_ASSERT(m_gasStiffness > 0.0) <<
		"The gas stiffness needs to be set prior to adding the component in the SceneElement";
	SURGSIM_ASSERT(m_h > 0.0) <<
		"The kernel support needs to be set prior to adding the component in the SceneElement";

	m_normal.resize(m_maxParticles, 3);
	m_acceleration.resize(m_maxParticles, 3);
	m_density.resize(m_maxParticles);
	m_pressure.resize(m_maxParticles);

	// The 3d grid is composed of 2^10 cubic cells on each dimension of size m_h each.
	// This covers a volume of (m_h * 2^10)^3 cubic meter centered on the origin.
	Math::Vector3d aabbSize(1024 * m_h, 1024 * m_h, 1024 * m_h);
	Eigen::AlignedBox<double, 3> aabb;
	aabb.min() = -aabbSize / 2.0;
	aabb.max() = aabbSize / 2.0;
	Math::Vector3d cellSize = Math::Vector3d::Constant(m_h);
	m_grid = std::make_shared<DataStructures::Grid<size_t, 3>>(cellSize, aabb);

	return true;
}

bool SphRepresentation::doUpdate(double dt)
{
	// Compute acceleration
	computeNeighbors();
	computeDensityAndPressureField();
	computeNormalField();
	computeAccelerations();

	// Integrate ODE to determine new velocity and position
	computeVelocityAndPosition(dt);

	return true;
}

void SphRepresentation::computeVelocityAndPosition(double dt)
{
	auto& particles = m_particles.unsafeGet().getVertices();
	for (size_t i = 0; i < particles.size(); i++)
	{
		particles[i].data.velocity += dt * m_acceleration.row(i);
		particles[i].position += dt * particles[i].data.velocity;
	}
}

void SphRepresentation::computeNeighbors()
{
	m_grid->reset();

	auto&  particles = m_particles.unsafeGet().getVertices();
	for (size_t i = 0; i < particles.size(); i++)
	{
		m_grid->addElement(i, particles[i].position);
	}
}

void SphRepresentation::computeDensityAndPressureField()
{
	auto& particles = m_particles.unsafeGet().getVertices();
	m_density.head(particles.size()).setZero();
	for (size_t i = 0; i < particles.size(); i++)
	{
		for (auto j : m_grid->getNeighbors(i))
		{
			const Math::Vector3d r = particles[i].position - particles[j].position;
			const double rSquaredNorm = r.squaredNorm();
			if (rSquaredNorm < m_hSquared)
			{
				m_density[i] += (m_hSquared - rSquaredNorm) * (m_hSquared - rSquaredNorm) * (m_hSquared - rSquaredNorm);
			}
		}
	}
	m_density *= m_mass * m_kernelPoly6;
	m_pressure = m_gasStiffness * (m_density.array() - m_densityReference);
}

void SphRepresentation::computeNormalField()
{
	auto& particles = m_particles.unsafeGet().getVertices();
	m_normal.topRows(particles.size()).setZero();
	for (size_t i = 0; i < particles.size(); i++)
	{
		for (auto j : m_grid->getNeighbors(i))
		{
			const Math::Vector3d r = particles[i].position - particles[j].position;
			const double rSquaredNorm = r.squaredNorm();
			if (rSquaredNorm < m_hSquared)
			{
				m_normal.row(i) += (m_hSquared - rSquaredNorm) *  (m_hSquared - rSquaredNorm)/ m_density[j] * r;
			}
		}
	}
	m_normal *= m_kernelPoly6Gradient * m_mass;
}

void SphRepresentation::computeAccelerations()
{
	auto& particles = m_particles.unsafeGet().getVertices();
	m_acceleration.topRows(particles.size()).setZero();
	for (size_t i = 0; i < particles.size(); i++)
	{
		for (auto j : m_grid->getNeighbors(i))
		{
			// Consider symmetry here
			if (j <= i)
			{
				continue;
			}

			const Math::Vector3d r = particles[i].position - particles[j].position;
			const double rSquaredNorm = r.squaredNorm();
			if (rSquaredNorm < m_hSquared)
			{
				// Pressure force
				const double rNorm = std::max(std::sqrt(rSquaredNorm), 0.0001);
				const Math::Vector3d gradient = r * m_kernelSpikyGradient * (m_h - rNorm) * (m_h -rNorm) / rNorm;
				Math::Vector3d f = (-(m_pressure[i] + m_pressure[j]) / (2.0 * m_density[i])) * gradient;

				// Viscosity force
				const Math::Vector3d v = particles[i].data.velocity - particles[j].data.velocity;
				const double laplacian = m_kernelViscosityLaplacian * (1.0 - rNorm / m_h);
				f += -(m_viscosity * v / m_density[j]) * laplacian;

				// Surface tension force
				const double normalNorm = m_normal.row(j).norm();
				if (normalNorm > 20.0)
				{
					const Math::Vector3d unitNormal = m_normal.row(j) / normalNorm;
					double laplacianPoly6 = m_kernelPoly6Laplacian * (m_hSquared - rSquaredNorm);
					laplacianPoly6 *= (rSquaredNorm - 3.0 / 4.0 * (m_hSquared - rSquaredNorm));
					f += -m_surfaceTension / m_density[j] * laplacianPoly6 * unitNormal;
				}

				// Action/reaction application on the pair of particles
				m_acceleration.row(i) += f;
				m_acceleration.row(j) -= f;
			}
		}
	}
	m_acceleration.array().colwise() *=  m_mass / m_density.array();

	const Math::Vector3d localGravity = getPose().linear().inverse() * m_gravity;
	m_acceleration.rowwise() += localGravity.transpose();
}

bool SphRepresentation::doHandleCollisions(double dt, const SurgSim::Collision::ContactMapType& collisions)
{
	const Math::RigidTransform3d inversePose = getPose().inverse();
	auto& particles = m_particles.unsafeGet().getVertices();

	for (auto& collision : collisions)
	{
		for (auto& contact : collision.second)
		{
			Math::Vector3d normal = inversePose.linear() * contact->normal;
			size_t index = contact->penetrationPoints.first.index.getValue();
			auto& particle = particles[index];

			double velocityAlongNormal = particle.data.velocity.dot(normal);
			double forceIntensity = m_stiffness * contact->depth - m_damping * velocityAlongNormal;

			Math::Vector3d tangentVelocity = particle.data.velocity - velocityAlongNormal * normal;
			Math::Vector3d forceDirection = normal - m_friction * tangentVelocity.normalized();
			Math::Vector3d accelerationCorrection = (forceIntensity / m_mass) * forceDirection;

			particle.data.velocity += dt * accelerationCorrection;
			particle.position += dt * dt * accelerationCorrection;
		}
	}
	return true;
}

}; // namespace Particles
}; // namespace SurgSim
