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

#ifndef SURGSIM_PARTICLES_SPHREPRESENTATION_H
#define SURGSIM_PARTICLES_SPHREPRESENTATION_H

#include <vector>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/ParticleSystemRepresentation.h"

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

/// SphRepresentation is a ParticleSystemRepresentation dedicated to Smoothed-Particles Hydrodynamics (SPH).
/// This class is mostly based on these papers:
/// "Particle-Based Fluid Simulation for Interactive Applications", M. Muller, D. Charypar, M. Gross.
/// In Proceedings of ACM SIGGRAPH Symposium on Computer Animation (SCA) 2003, pp 154-159.
/// "Interactive Blood Simulation for Virtual Surgery Based on Smoothed Particle Hydrodynamics", M. Muller,
/// S. Schirm, M. Teschner. Journal of Technology and Health Care, ISSN 0928-7329, IOS Press, Amsterdam.
class SphRepresentation : public ParticleSystemRepresentation
{
public:
	SURGSIM_CLASSNAME(SurgSim::Particles::SphRepresentation);

	/// Data structure for constraining the fluid on the positive side of a plane
	struct PlaneConstraint
	{
		/// Constructor initializing default value for stiffness and damping
		PlaneConstraint() : stiffness(50000.0), damping(100.0)
		{}

		/// Plane equation \f$(n_x, n_y, n_z, d)\f$.
		/// Any plane point P verifies \f$\mathbf{P}.dot(\mathbf{n}) + d = 0\f$
		SurgSim::Math::Vector4d planeEquation;

		/// Collision stiffness
		double stiffness;

		/// Collision damping
		double damping;
	};

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

	/// Add a plane constraint
	/// \param planeConstraint to interact with the fluid
	void addPlaneConstraint(const PlaneConstraint& planeConstraint);

	/// Set all the plane constraints
	/// \param planeConstraints All the plane constraints interacting with the fluid
	void setPlaneConstraints(const std::vector<SphRepresentation::PlaneConstraint>& planeConstraints);

	/// Get all the plane constraints
	/// \return All plane constraints interacting with the fluid
	const std::vector<SphRepresentation::PlaneConstraint>& getPlaneConstraints() const;

protected:
	std::vector<SurgSim::Math::Vector3d> m_normal;  		///< Particles' normal
	std::vector<SurgSim::Math::Vector3d> m_acceleration;	///< Particles' acceleration
	std::vector<double> m_density;                  		///< Particles' density
	std::vector<double> m_pressure;                 		///< Particles' pressure
	std::vector<double> m_mass;                     		///< Particles' mass
	double m_massPerParticle;                       ///< Mass per particle (determine the density of particle per m3)
	double m_densityReference;                      ///< Density of the reference gas
	double m_gasStiffness;                          ///< Stiffness of the gas considered
	double m_surfaceTension;                        ///< Surface tension

	SurgSim::Math::Vector3d m_gravity;              ///< 3D Gravity vector
	double m_viscosity;                             ///< Viscosity coefficient

	/// Kernels parameter (support length and its powers)
	double m_h, m_hPower2, m_hPower3, m_hPower5, m_hPower6, m_hPower9;

	/// Grid acceleration to evaluate the kernels locally (storing the particles' index)
	std::shared_ptr<SurgSim::DataStructures::Grid<size_t, 3>> m_grid;

	/// Collision planes
	std::vector<PlaneConstraint> m_planeConstraints;

	bool doInitialize() override;

	bool doUpdate(double dt) override;

	/// Compute the particles' acceleration given a time step dt
	/// \param dt The time step to advance the simulation too
	/// \note This method stores the accelerations in the state.
	virtual void computeAcceleration(double dt);

	/// Compute the particles' velocity and position given a time step dt
	/// \param dt The time step to advance the simulation too
	/// \note This method integrates the ODE equation of the SPH, computing velocities and positions from the
	/// \note accelerations and storing them in the state. Therefore computeAcceleration(dt) should be called before.
	virtual void computeVelocityAndPosition(double dt);

private:
	/// Compute the neighbors
	virtual void computeNeighbors();

	/// Compute the density and pressure field
	virtual void computeDensityAndPressureField();

	/// Compute the normal field
	virtual void computeNormalField();

	/// Compute the Sph accelerations
	virtual void computeAccelerations();

	/// Compute collision detection and response against all plane constraints
	virtual void handleCollisions();

	/// Kernel poly6
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel poly6 evaluated with rij and m_h
	double kernelPoly6(const SurgSim::Math::Vector3d& rij);

	/// Kernel poly6's gradient
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel poly6's gradient evaluated with rij and m_h
	SurgSim::Math::Vector3d kernelPoly6Gradient(const SurgSim::Math::Vector3d& rij);

	/// Kernel poly6's laplacian
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel poly6's laplacian evaluated with rij and m_h
	double kernelPoly6Laplacian(const SurgSim::Math::Vector3d& rij);

	/// Kernel spiky
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel spiky evaluated with rij and m_h
	double kernelSpiky(const SurgSim::Math::Vector3d& rij);

	/// Kernel spiky's gradient
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel spiky's gradient evaluated with rij and m_h
	SurgSim::Math::Vector3d kernelSpikyGradient(const SurgSim::Math::Vector3d& rij);

	/// Kernel viscosity
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel viscosity evaluated with rij and m_h
	double kernelViscosity(const SurgSim::Math::Vector3d& rij);

	/// Kernel viscosity's gradient
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel viscosity's gradient evaluated with rij and m_h
	SurgSim::Math::Vector3d kernelViscosityGradient(const SurgSim::Math::Vector3d& rij);

	/// Kernel viscosity's laplacian
	/// \param rij The vector between the 2 particles considered \f$r_i - r_j\f$
	/// \return The kernel viscosity's laplacian evaluated with rij and m_h
	double kernelViscosityLaplacian(const SurgSim::Math::Vector3d& rij);
};

};  // namespace Particles
};  // namespace SurgSim

namespace YAML
{
/// Specialization of YAML::convert for SphRepresentation::PlaneConstraint
template <>
struct convert<SurgSim::Particles::SphRepresentation::PlaneConstraint>
{
	static Node encode(const SurgSim::Particles::SphRepresentation::PlaneConstraint rhs)
	{
		Node data, result;
		data["PlaneEquation"] = rhs.planeEquation;
		data["Stiffness"] = rhs.stiffness;
		data["Damping"] = rhs.damping;
		result["PlaneConstraint"] = data;
		return result;
	}
	static bool decode(const Node& node, SurgSim::Particles::SphRepresentation::PlaneConstraint& rhs) //NOLINT
	{
		try
		{
			Node data = node["PlaneConstraint"];
			rhs.planeEquation = data["PlaneEquation"].as<SurgSim::Math::Vector4d>();
			rhs.stiffness= data["Stiffness"].as<double>();
			rhs.damping = data["Damping"].as<double>();
		}
		catch (YAML::RepresentationException)
		{
			auto logger = SurgSim::Framework::Logger::getLogger("Particles");
			SURGSIM_LOG(logger, WARNING) << "Bad conversion to SphRepresentation::PlaneConstraint";
			return false;
		}
		return true;
	}
};
}; // namespace YAML

#endif  // SURGSIM_PARTICLES_SPHREPRESENTATION_H
