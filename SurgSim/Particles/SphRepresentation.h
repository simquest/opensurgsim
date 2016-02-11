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

#ifndef SURGSIM_PARTICLES_SPHREPRESENTATION_H
#define SURGSIM_PARTICLES_SPHREPRESENTATION_H

#include <Eigen/Core>
#include <vector>

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/Representation.h"


namespace SurgSim
{

namespace DataStructures
{
template <class T, size_t N>
class Grid;
}; // namespace DataStructures

namespace Particles
{

SURGSIM_STATIC_REGISTRATION(SphRepresentation);

/// SphRepresentation is a Representation dedicated to Smoothed-Particles Hydrodynamics (SPH).
/// This class is mostly based on these papers:
/// "Particle-Based Fluid Simulation for Interactive Applications", M. Muller, D. Charypar, M. Gross.
/// In Proceedings of ACM SIGGRAPH Symposium on Computer Animation (SCA) 2003, pp 154-159.
/// "Interactive Blood Simulation for Virtual Surgery Based on Smoothed Particle Hydrodynamics", M. Muller,
/// S. Schirm, M. Teschner. Journal of Technology and Health Care, ISSN 0928-7329, IOS Press, Amsterdam.
class SphRepresentation : public Representation
{
public:
	SURGSIM_CLASSNAME(SurgSim::Particles::SphRepresentation);

	/// Constructor
	/// \param name The representation's name
	explicit SphRepresentation(const std::string& name);

	/// Destructor
	virtual ~SphRepresentation();

	/// Set the mass for each particle
	/// \param particleMass The mass that will be used for all particles [Kg]
	/// \throws An exception SurgSim::Framework::AssertionFailure if the value is negative or null
	/// \note In the SPH model, a particle has a constant mass, but its volume and density vary
	/// \note (mass = volume * density). <br>
	/// \note Example: If we want to simulate 1 liter of water (0.001 m3 at 1000kg.m-3)
	/// \note with 50 particles, we need each particle to have a mass of 1.0/50 = 0.02kg
	void setMassPerParticle(double particleMass);

	/// Get the mass for each particle
	/// \return The mass that is used for all particle [Kg]
	double getMassPerParticle() const;

	/// Set the density of the fluid
	/// \param density of the fluid [Kg.m-3]
	/// \throws An exception SurgSim::Framework::AssertionFailure if the value is negative or null
	void setDensity(double density);

	/// Get the density of the fluid
	/// \return The density of the fluid [Kg.m-3]
	double getDensity() const;

	/// Set the gas stiffness coefficient
	/// \param stiffness coefficient of the gas [N.m.Kg-1]
	/// \throws An exception SurgSim::Framework::AssertionFailure if the value is negative or null
	void setGasStiffness(double stiffness);

	/// Get the gas stiffness coefficient
	/// \return The stiffness coefficient of the gas [N.m.Kg-1]
	double getGasStiffness() const;

	/// Set the surface tension
	/// \param surfaceTension The surface tension [N.m-1]
	/// \throws An exception SurgSim::Framework::AssertionFailure if the value is negative
	void setSurfaceTension(double surfaceTension);

	/// Get the surface tension
	/// \return The surface tension [N.m-1]
	double getSurfaceTension() const;

	/// Set the gravity vector
	/// \param gravity The 3d gravity vector [m]
	void setGravity(const SurgSim::Math::Vector3d& gravity);

	/// Get the gravity vector (default is (0 -9.81 0))
	/// \return The 3d gravity vector [m]
	SurgSim::Math::Vector3d getGravity() const;

	/// Set the viscosity coefficient
	/// \param viscosity coefficient [N.s.m-2]
	/// \throws An exception SurgSim::Framework::AssertionFailure if the value is negative
	void setViscosity(double viscosity);

	/// Get the viscosity coefficient (default is 0.0)
	/// \return The viscosity coefficient [N.s.m-2]
	double getViscosity() const;

	/// Set the kernel function support
	/// \param support The length of the kernel support [m]
	/// \throws An exception SurgSim::Framework::AssertionFailure if the value is negative or null
	void setKernelSupport(double support);

	/// Get the kernel function support
	/// \return The length of the kernel support [m]
	double getKernelSupport() const;

	/// Set the particles stiffness when colliding
	/// \param stiffness The stiffness [N/m]
	void setStiffness(double stiffness);

	/// Get the particles stiffness when colliding
	/// \return The stiffness [N/m]
	double getStiffness() const;

	/// Set the particles damping when colliding
	/// \param damping The damping [Ns/m]
	void setDamping(double damping);

	/// Get the particles damping when colliding
	/// \return The damping [Ns/m]
	double getDamping() const;

	/// Set the sliding coefficient of friction for the particles during collisions
	/// \param friction The sliding coefficient of friction
	void setFriction(double friction);

	/// Get the sliding coefficient of friction for the particles during collisions
	/// \return The sliding coefficient of friction
	double getFriction() const;

protected:
	bool doInitialize() override;

	bool doUpdate(double dt) override;

	bool doHandleCollisions(double dt, const SurgSim::Collision::ContactMapType& collisions) override;

	/// Compute the particles' velocity and position given a time step dt
	/// \param dt The time step to advance the simulation too
	/// \note This method integrates the ODE equation of the SPH, computing velocities and positions from the
	/// \note accelerations and storing them in the state. Therefore computeAcceleration(dt) should be called before.
	void computeVelocityAndPosition(double dt);

	Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> m_normal;			///< Particles' normal
	Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> m_acceleration;	///< Particles' acceleration
	Math::Vector m_density;                  		///< Particles' density
	Math::Vector m_pressure;                 		///< Particles' pressure
	double m_mass;                       			///< Mass per particle (determine the density of particle per m3)
	double m_densityReference;                      ///< Density of the reference gas
	double m_gasStiffness;                          ///< Stiffness of the gas considered
	double m_surfaceTension;                        ///< Surface tension
	double m_stiffness;                             ///< Collision stiffness
	double m_damping;                               ///< Collision damping
	double m_friction;                              ///< Collision sliding friction coefficient
	SurgSim::Math::Vector3d m_gravity;              ///< 3D Gravity vector
	double m_viscosity;                             ///< Viscosity coefficient

	/// Kernels parameter (support length and its powers)
	double m_h;
	double m_hSquared;
	double m_kernelPoly6;
	double m_kernelPoly6Gradient;
	double m_kernelSpikyGradient;
	double m_kernelViscosityLaplacian;
	double m_kernelPoly6Laplacian;

	/// Grid acceleration to evaluate the kernels locally (storing the particles' index)
	std::shared_ptr<SurgSim::DataStructures::Grid<size_t, 3>> m_grid;

private:
	/// Compute the neighbors
	void computeNeighbors();

	/// Compute the density and pressure field
	void computeDensityAndPressureField();

	/// Compute the normal field
	void computeNormalField();

	/// Compute the Sph accelerations
	void computeAccelerations();

};

};  // namespace Particles
};  // namespace SurgSim

#endif  // SURGSIM_PARTICLES_SPHREPRESENTATION_H
