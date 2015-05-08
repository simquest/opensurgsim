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

#include "SurgSim/Particles/SphRepresentation.h"

#include "SurgSim/DataStructures/Grid.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::Vector;

namespace SurgSim
{
namespace Particles
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Particles::SphRepresentation, SphRepresentation);

SphRepresentation::SphRepresentation(const std::string& name) :
	ParticleSystemRepresentation(name),
	m_massPerParticle(0.0),
	m_densityReference(0.0),
	m_gasStiffness(0.0),
	m_surfaceTension(0.0),
	m_gravity(SurgSim::Math::Vector3d(0.0, -9.81, 0.0)),
	m_viscosity(0.0),
	m_h(0.0)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double,
		MassPerParticle, getMassPerParticle, setMassPerParticle);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double,
		Density, getDensity, setDensity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double,
		GasStiffness, getGasStiffness, setGasStiffness);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double,
		SurfaceTension, getSurfaceTension, setSurfaceTension);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double,
		Viscosity, getViscosity, setViscosity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, double,
		KernelSupport, getKernelSupport, setKernelSupport);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, SurgSim::Math::Vector3d,
		Gravity, getGravity, setGravity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SphRepresentation, std::vector<PlaneConstraint>,
		PlaneConstraints, getPlaneConstraints, setPlaneConstraints);
}

SphRepresentation::~SphRepresentation()
{
}

void SphRepresentation::setMassPerParticle(double particleMass)
{
	SURGSIM_ASSERT(particleMass > 0.0) <<
		"The mass per particle needs to be a valid positive non null value." << particleMass << " was provided.";
	m_massPerParticle = particleMass;
}

double SphRepresentation::getMassPerParticle() const
{
	return m_massPerParticle;
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
	m_hPower2 = m_h * m_h;
	m_hPower3 = m_hPower2 * m_h;
	m_hPower5 = m_hPower3 * m_hPower2;
	m_hPower6 = m_hPower5 * m_h;
	m_hPower9 = m_hPower6 * m_hPower3;
}

double SphRepresentation::getKernelSupport() const
{
	return m_h;
}

void SphRepresentation::addPlaneConstraint(const PlaneConstraint& planeConstraint)
{
	m_planeConstraints.push_back(planeConstraint);
}

void SphRepresentation::setPlaneConstraints(const std::vector<SphRepresentation::PlaneConstraint>& planeConstraints)
{
	m_planeConstraints = planeConstraints;
}

const std::vector<SphRepresentation::PlaneConstraint>& SphRepresentation::getPlaneConstraints() const
{
	return m_planeConstraints;
}

bool SphRepresentation::doInitialize()
{
	if (!ParticleSystemRepresentation::doInitialize())
	{
		return false;
	}

	SURGSIM_ASSERT(m_massPerParticle > 0.0) <<
		"The mass per particle needs to be set prior to adding the component in the SceneElement";
	SURGSIM_ASSERT(m_densityReference > 0.0) <<
		"The reference density needs to be set prior to adding the component in the SceneElement";
	SURGSIM_ASSERT(m_gasStiffness > 0.0) <<
		"The gas stiffness needs to be set prior to adding the component in the SceneElement";
	SURGSIM_ASSERT(m_h > 0.0) <<
		"The kernel support needs to be set prior to adding the component in the SceneElement";

	m_normal.resize(m_maxParticles);
	m_acceleration.resize(m_maxParticles);
	m_density.resize(m_maxParticles);
	m_pressure.resize(m_maxParticles);
	m_mass.resize(m_maxParticles);
	m_mass.assign(m_mass.size(), m_massPerParticle);

	// The 3d grid is composed of 2^10 cubic cells on each dimension of size m_h each.
	// This covers a volume of (m_h * 2^10)^3 cubic meter centered on the origin.
	SurgSim::Math::Vector3d aabbSize(1024 * m_h, 1024 * m_h, 1024 * m_h);
	Eigen::AlignedBox<double, 3> aabb;
	aabb.min() = -aabbSize / 2.0;
	aabb.max() = aabbSize / 2.0;
	SurgSim::Math::Vector3d cellSize = SurgSim::Math::Vector3d::Constant(m_h);
	m_grid = std::make_shared<SurgSim::DataStructures::Grid<size_t, 3>>(cellSize, aabb);

	return true;
}

bool SphRepresentation::doUpdate(double dt)
{
	// Compute acceleration
	computeAcceleration(dt);

	// Handle the collisions by affecting the accelerations
	handleCollisions();

	// Integrate ODE to determine new velocity and position
	computeVelocityAndPosition(dt);

	return true;
}

void SphRepresentation::computeAcceleration(double dt)
{
	computeNeighbors();
	computeDensityAndPressureField();
	computeNormalField();
	computeAccelerations();
}

void SphRepresentation::computeVelocityAndPosition(double dt)
{
	auto acceleration = m_acceleration.begin();
	for (auto& particle : getParticles().getVertices())
	{
		particle.data.velocity += dt * (*acceleration);
		particle.position += dt * particle.data.velocity;
		acceleration++;
	}
}

void SphRepresentation::computeNeighbors()
{
	m_grid->reset();

	size_t index = 0;
	for (auto const& particle : getParticles().getVertices())
	{
		m_grid->addElement(index, particle.position);
		index++;
	}
}

void SphRepresentation::computeDensityAndPressureField()
{
	size_t indexI = 0;
	auto& particles = m_particles.getVertices();
	for (auto& particleI : particles)
	{
		// Calculate the particle's density
		double densityI = 0.0;
		for (auto indexJ : m_grid->getNeighbors(indexI))
		{
			densityI += m_mass[indexJ] * kernelPoly6(particleI.position - particles[indexJ].position);
		}
		m_density[indexI] = densityI;

		// Calculate the particle's pressure
		m_pressure[indexI] = m_gasStiffness * (m_density[indexI] - m_densityReference);
		indexI++;
	}
}

void SphRepresentation::computeNormalField()
{
	auto& particles = m_particles.getVertices();
	size_t indexI = 0;
	SurgSim::Math::Vector3d normalI;
	SurgSim::Math::Vector3d gradient;
	for (auto& particleI : particles)
	{
		// Calculate the particle's normal (gradient of the color field)
		normalI = SurgSim::Math::Vector3d::Zero();
		for (auto indexJ : m_grid->getNeighbors(indexI))
		{
			gradient = kernelPoly6Gradient(particleI.position - particles[indexJ].position);
			normalI += m_mass[indexJ] / m_density[indexJ] * gradient;
		}
		m_normal[indexI] = normalI;
		indexI++;
	}
}

void SphRepresentation::computeAccelerations()
{
	auto& particles = m_particles.getVertices();
	for (size_t i = 0; i < particles.size(); i++)
	{
		m_acceleration[i] = SurgSim::Math::Vector3d::Zero();
	}

	size_t indexI = 0;
	SurgSim::Math::Vector3d f;
	SurgSim::Math::Vector3d gradient;
	SurgSim::Math::Vector3d unitNormal;
	for (auto& particleI : particles)
	{
		for (auto indexJ : m_grid->getNeighbors(indexI))
		{
			// Consider symmetry here
			if (indexJ <= indexI)
			{
				continue;
			}
			Particle& particleJ = particles[indexJ];

			SurgSim::Math::Vector3d rij = particleI.position - particleJ.position;
			SurgSim::Math::Vector3d vji = -particleI.data.velocity + particleJ.data.velocity;

			// Pressure force
			gradient = kernelSpikyGradient(rij);
			f = (-m_mass[indexJ] * (m_pressure[indexI] + m_pressure[indexJ]) / (2.0 * m_density[indexI])) * gradient;

			// Viscosity force
			double laplacian = kernelViscosityLaplacian(rij);
			f += (m_viscosity * m_mass[indexJ] * vji / m_density[indexJ]) * laplacian;

			// Surface tension force
			double laplacianPoly6 = kernelPoly6Laplacian(rij);
			double normalNorm = m_normal[indexJ].norm();
			if (normalNorm > 20.0)
			{
				unitNormal = m_normal[indexJ] / normalNorm;
				f += -m_surfaceTension * m_mass[indexJ] / m_density[indexJ] * laplacianPoly6 * unitNormal;
			}

			// Action/reaction application on the pair of particles
			m_acceleration[indexI] += f;
			m_acceleration[indexJ] -= f;
		}

		// Compute the acceleration from the forces
		m_acceleration[indexI] /= m_density[indexI];

		// Adding the gravity term (F = rho.g)
		m_acceleration[indexI] += m_gravity;

		indexI++;
	}
}

void SphRepresentation::handleCollisions()
{
	for (auto planeConstraint : m_planeConstraints)
	{
		auto normal = planeConstraint.planeEquation.segment<3>(0);

		size_t index = 0;
		for (auto& particle : m_particles.getVertices())
		{
			double penetration = particle.position.dot(normal) + planeConstraint.planeEquation[3];
			if (penetration < 0.0)
			{
				double forceIntensity = planeConstraint.stiffness * penetration;
				forceIntensity += planeConstraint.damping * particle.data.velocity.dot(normal);
				m_acceleration[index] -= forceIntensity * normal;
			}
			index++;
		}
	}
}

double SphRepresentation::kernelPoly6(const SurgSim::Math::Vector3d& rij)
{
	double rPower2 = rij.squaredNorm();

	if (rPower2 <= m_hPower2)
	{
		double hh_minus_rr = (m_hPower2 - rPower2);
		return 315.0 / (64.0 * M_PI * m_hPower9) * hh_minus_rr * hh_minus_rr * hh_minus_rr;
	}
	else
	{
		return 0.0;
	}
}

SurgSim::Math::Vector3d SphRepresentation::kernelPoly6Gradient(const SurgSim::Math::Vector3d& rij)
{
	double rPower2 = rij.squaredNorm();

	if (rPower2 <= m_hPower2)
	{
		double hh_minus_rr = (m_hPower2 - rPower2);
		return -rij * 945.0 / (32.0 * M_PI * m_hPower9) * hh_minus_rr * hh_minus_rr;
	}
	else
	{
		return SurgSim::Math::Vector3d::Zero();
	}
}

double SphRepresentation::kernelPoly6Laplacian(const SurgSim::Math::Vector3d& rij)
{
	double rPower2 = rij.squaredNorm();

	if (rPower2 <= m_hPower2)
	{
		double hh_minus_rr = (m_hPower2 - rPower2);
		return 945.0 / (8.0 * M_PI * m_hPower9) * hh_minus_rr * (rPower2 - 3.0 / 4.0 * hh_minus_rr);
	}
	else
	{
		return 0.0;
	}
}

double SphRepresentation::kernelSpiky(const SurgSim::Math::Vector3d& rij)
{
	double r = rij.norm();

	if (r <= m_h)
	{
		double h_minus_r = m_h - r;
		return 15.0 / (M_PI * m_hPower6) * h_minus_r * h_minus_r * h_minus_r;
	}
	else
	{
		return 0.0;
	}
}

SurgSim::Math::Vector3d SphRepresentation::kernelSpikyGradient(const SurgSim::Math::Vector3d& rij)
{
	double r = rij.norm();

	if (r <= m_h)
	{
		double h_minus_r = m_h - r;
		return -rij * 45.0 / (M_PI * m_hPower6 * r) * h_minus_r * h_minus_r;
	}
	else
	{
		return SurgSim::Math::Vector3d::Zero();
	}
}

double SphRepresentation::kernelViscosity(const SurgSim::Math::Vector3d& rij)
{
	double r = rij.norm();

	if (r <= m_h)
	{
		double q = r / m_h;
		double qPower2 = q * q;
		return 15.0 / (2.0 * M_PI * m_hPower3) * (-0.5 * q * qPower2 + qPower2 + 0.5 / q - 1.0);
	}
	else
	{
		return 0.0;
	}
}

SurgSim::Math::Vector3d SphRepresentation::kernelViscosityGradient(const SurgSim::Math::Vector3d& rij)
{
	double r = rij.norm();

	if (r <= m_h)
	{
		return rij * 15.0 / (2.0 * M_PI * m_hPower3) * (-1.5 * r / m_hPower3 + 2.0 / m_hPower2 - 0.5 * m_h / (r*r*r));
	}
	else
	{
		return SurgSim::Math::Vector3d::Zero();
	}
}

double SphRepresentation::kernelViscosityLaplacian(const SurgSim::Math::Vector3d& rij)
{
	double r = rij.norm();

	if (r <= m_h)
	{
		return 45.0 / (M_PI * m_hPower5) * (1.0 - r / m_h);
	}
	else
	{
		return 0.0;
	}
}

}; // namespace Particles
}; // namespace SurgSim
