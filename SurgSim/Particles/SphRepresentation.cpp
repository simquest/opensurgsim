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
#include "SurgSim/Particles/Particle.h"
#include "SurgSim/Particles/ParticleReference.h"
#include "SurgSim/Particles/ParticlesState.h"

namespace SurgSim
{
namespace Particles
{

SphRepresentation::SphRepresentation(const std::string& name) :
	ParticleSystemRepresentation(name),
	m_massPerParticle(0.0),
	m_densityReference(0.0),
	m_gasStiffness(0.0),
	m_surfaceTensionCoefficient(0.0),
	m_viscosity(0.0),
	m_h(0.0)
{
	m_gravity = SurgSim::Math::Vector3d(0.0, -9.81, 0.0);
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

void SphRepresentation::setDensityReference(double density)
{
	SURGSIM_ASSERT(density > 0.0) <<
		"The reference density needs to be a valid positive non null value." << density << " was provided.";
	m_densityReference = density;
}

double SphRepresentation::getDensityReference() const
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

void SphRepresentation::setSurfaceTensionCoefficient(double surfaceTensionCoefficient)
{
	SURGSIM_ASSERT(surfaceTensionCoefficient >= 0.0) <<
		"The surface tension coefficient needs to be a valid positive or null value." <<
		surfaceTensionCoefficient << " was provided.";
	m_surfaceTensionCoefficient = surfaceTensionCoefficient;
}

double SphRepresentation::getSurfaceTensionCoefficient() const
{
	return m_surfaceTensionCoefficient;
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
	m_hPow2 = m_h * m_h;
	m_hPow3 = m_hPow2 * m_h;
	m_hPow5 = m_hPow3 * m_hPow2;
	m_hPow6 = m_hPow5 * m_h;
	m_hPow9 = m_hPow6 * m_hPow3;
}

double SphRepresentation::getKernelSupport() const
{
	return m_h;
}

void SphRepresentation::addPlaneConstraint(const PlaneConstraint& planeConstraint)
{
	m_planeConstraints.push_back(planeConstraint);
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

	m_a.setZero(3 * m_maxParticles);
	m_normal.resize(m_maxParticles);
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
	auto& x = m_state->getPositions();
	auto& v = m_state->getVelocities();

	v += dt * m_a;
	x += dt * v;
}

void SphRepresentation::computeNeighbors()
{
	m_grid->reset();

	for (auto const& particleI : getParticleReferences())
	{
		m_grid->addElement(particleI.getIndex(), particleI.getPosition());
	}
}

void SphRepresentation::computeDensityAndPressureField(void)
{
	m_density.assign(m_density.size(), 0.0);
	m_pressure.assign(m_density.size(), 0.0);

	for (std::list<ParticleReference>::iterator particleI = getParticleReferences().begin();
		particleI != getParticleReferences().end();
		++particleI)
	{
		size_t indexI = particleI->getIndex();
		double &rhoI = m_density[indexI];
		double &pI = m_pressure[indexI];
		const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xI = particleI->getPosition();

		// Calculate the particle's density
		for (auto indexJ : m_grid->getNeighbors(indexI))
		{
			const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xJ = m_state->getPositions().segment<3>(3 * indexJ);
			rhoI += m_mass[indexJ] * KernelPoly6(xI - xJ);
		}

		// Calculate the particle's pressure
		pI = m_gasStiffness * (rhoI - m_densityReference);
	}
}

void SphRepresentation::computeNormalField(void)
{
	m_normal.assign(m_normal.size(), SurgSim::Math::Vector3d::Zero());

	for (std::list<ParticleReference>::iterator particleI = getParticleReferences().begin();
		particleI != getParticleReferences().end();
		++particleI)
	{
		const size_t indexI = particleI->getIndex();
		SurgSim::Math::Vector3d& normalI = m_normal[indexI];
		const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xI = particleI->getPosition();

		// Calculate the particle's normal (gradient of the color field)
		for (auto indexJ : m_grid->getNeighbors(indexI))
		{
			const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xJ = m_state->getPositions().segment<3>(3 * indexJ);
			normalI += m_mass[indexJ] / m_density[indexJ] * KernelPoly6Gradient(xI - xJ);
		}
	}
}

void SphRepresentation::computeAccelerations(void)
{
	SurgSim::Math::Vector3d f;

	for (std::list<ParticleReference>::iterator particleI = getParticleReferences().begin();
		particleI != getParticleReferences().end();
		++particleI)
	{
		const size_t indexI = particleI->getIndex();
		const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xI = particleI->getPosition();
		const Eigen::VectorBlock<SurgSim::Math::Vector, 3> vI = particleI->getVelocity();
		Eigen::VectorBlock<SurgSim::Math::Vector, 3> aI = m_a.segment<3>(3 * indexI);
		const double &rhoI = m_density[indexI];
		const double &pI = m_pressure[indexI];

		for (auto indexJ : m_grid->getNeighbors(indexI))
		{
			// Consider symmetry here
			if (indexJ <= indexI)
			{
				continue;
			}
			const double& massJ = m_mass[indexJ];
			const double& rhoJ = m_density[indexJ];
			const double& pJ = m_pressure[indexJ];
			const SurgSim::Math::Vector3d& normalJ = m_normal[indexJ];
			const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xJ = m_state->getPositions().segment<3>(3 * indexJ);
			const Eigen::VectorBlock<SurgSim::Math::Vector, 3> vJ = m_state->getVelocities().segment<3>(3 * indexJ);
			Eigen::VectorBlock<SurgSim::Math::Vector, 3> aJ = m_a.segment<3>(3 * indexJ);

			// Acceleration from the pressure
			SurgSim::Math::Vector3d grad = KernelSpikyGradient(xI - xJ);
			f = (-massJ * (pI + pJ) / (2.0 * rhoI)) * grad;

			// Acceleration from the viscosity
			double laplacian = KernelViscosityLaplacian(xI - xJ);
			f += (m_viscosity * massJ * (vJ - vI) / rhoJ) * laplacian;

			// Acceleration from the surface tension
			double laplacianPoly6 = KernelPoly6Laplacian(xI - xJ);
			double normalNorm = normalJ.norm();
			if (normalNorm > 20.0)
			{
				f += -m_surfaceTensionCoefficient * massJ / rhoJ * laplacianPoly6 * normalJ.normalized();
			}

			aI += f;
			aJ -= f;
		}

		// Compute the acceleration from the forces
		aI /= rhoI;

		// Adding the gravity term (F = rho.g)
		aI += m_gravity;
	}
}

void SphRepresentation::handleCollisions()
{
	for (auto planeConstraint : m_planeConstraints)
	{
		auto n = planeConstraint.planeEquation.segment<3>(0);

		for (auto const & particleI : getParticleReferences())
		{
			const Eigen::VectorBlock<SurgSim::Math::Vector, 3> xI = particleI.getPosition();
			double penetration = xI.dot(n) + planeConstraint.planeEquation[3];

			if (penetration < 0.0)
			{
				const Eigen::VectorBlock<SurgSim::Math::Vector, 3> vI = particleI.getVelocity();
				double forceIntensity = (planeConstraint.stiffness * penetration + planeConstraint.damping * vI.dot(n));
				m_a.segment<3>(3 * particleI.getIndex()) -= forceIntensity * n;
			}
		}
	}
}

}; // namespace Particles
}; // namespace SurgSim
