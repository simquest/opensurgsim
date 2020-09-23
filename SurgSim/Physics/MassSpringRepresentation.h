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

#ifndef SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H
#define SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H

#include <memory>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace SurgSim
{
namespace Framework
{
class Asset;
}

namespace Physics
{
class Mass;
class MassSpring;
class Spring;

SURGSIM_STATIC_REGISTRATION(MassSpringRepresentation);

/// MassSpring model is a deformable model (a set of masses connected by springs).
/// \note A MassSpring is a DeformableRepresentation (Physics::Representation and Math::OdeEquation)
/// \note Therefore, it defines a dynamic system M.a=F(x,v) with the particularity that M is diagonal
/// \note The model handles damping through the Rayleigh damping (where damping is a combination of mass and stiffness)
class MassSpringRepresentation : public DeformableRepresentation
{
public:
	/// Constructor
	/// \param name The name of the MassSpringRepresentation
	explicit MassSpringRepresentation(const std::string& name);

	/// Destructor
	virtual ~MassSpringRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::MassSpringRepresentation);

	/// Adds a mass
	/// \param mass The mass to add to the representation
	/// \note Masses are kept in an ordered list, giving them an index
	/// \note This mass will be associated with the node of same index in any associated OdeState
	void addMass(const std::shared_ptr<Mass> mass);

	/// Adds a spring
	/// \param spring The spring to add to the representation
	void addSpring(const std::shared_ptr<Spring> spring);

	/// Gets the number of masses
	/// \return the number of masses
	size_t getNumMasses() const;

	/// Gets the number of springs
	/// \return the number of springs
	size_t getNumSprings() const;

	/// Retrieves the mass of a given node
	/// \param nodeId The node id for which the mass is requested
	/// \return the mass attribute of a node
	/// \note The mass is returned with read/write access
	/// \note Out of range nodeId will raise an exception
	std::shared_ptr<Mass> getMass(size_t nodeId);

	/// Retrieves a given spring from its id
	/// \param springId The spring id for which the spring is requested
	/// \return the spring for the given springId
	/// \note The spring is returned with read/write access
	/// \note Out of range springId will raise an exception
	std::shared_ptr<Spring> getSpring(size_t springId);

	/// Gets the total mass of the mass spring
	/// \return The total mass of the mass spring (in Kg)
	double getTotalMass() const;

	/// Gets the Rayleigh stiffness parameter
	/// \return The Rayleigh stiffness parameter
	double getRayleighDampingStiffness() const;

	/// Gets the Rayleigh mass parameter
	/// \return The Rayleigh mass parameter
	double getRayleighDampingMass() const;

	/// Sets the Rayleigh stiffness parameter
	/// \param stiffnessCoef The Rayleigh stiffness parameter
	void setRayleighDampingStiffness(double stiffnessCoef);

	/// Sets the Rayleigh mass parameter
	/// \param massCoef The Rayleigh mass parameter
	void setRayleighDampingMass(double massCoef);

	void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
									 const SurgSim::Math::Vector& generalizedForce,
									 const SurgSim::Math::Matrix& K = SurgSim::Math::Matrix(),
									 const SurgSim::Math::Matrix& D = SurgSim::Math::Matrix()) override;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt) override;

	std::shared_ptr<Localization> createLocalization(const SurgSim::DataStructures::Location& location) override;

	/// Loads a MassSpringRepresentation from a ply file.
	/// \param filename The name of the file.
	void loadMassSpring(const std::string & filename);

	/// Sets the mesh asset
	/// \param mesh The mesh to assign to this representation
	/// \exception SurgSim::Framework::AssertionFailure if mesh is nullptr or it's actual type is not MassSpring
	void setMassSpring(std::shared_ptr<Framework::Asset> mesh);

	/// \return The mesh asset as a MassSpring.
	std::shared_ptr<MassSpring> getMassSpring() const;

protected:
	/// Add the Rayleigh damping forces
	/// \param[in,out] f The force vector to cumulate the Rayleigh damping force into
	/// \param state The state vector containing positions and velocities
	/// \param useGlobalMassMatrix, useGlobalStiffnessMatrix True indicates that the global mass and stiffness matrices
	///        should be used (F = -c.M.v - d.K.v)
	/// \param scale A scaling factor to apply on the damping force
	/// \note Damping matrix D = c.M + d.K (Rayleigh damping definition)
	/// \note F = - D.v = -c.M.v - d.K.v
	/// \note If {useGlobalMassMatrix | useGlobalStiffnessMatrix} is True, {M | K} will be used, otherwise
	/// \note    the {mass|stiffness} component will be computed FemElement by FemElement
	void addRayleighDampingForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state,
								 bool useGlobalStiffnessMatrix = false, bool useGlobalMassMatrix = false,
								 double scale = 1.0);

	/// Add the springs force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the spring forces into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the spring forces with
	void addSpringsForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state, double scale = 1.0);

	/// Add the gravity force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the gravity force into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the gravity force with
	/// \note This method does not do anything if gravity is disabled
	void addGravityForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state, double scale = 1.0);

	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
						const SurgSim::Math::RigidTransform3d& transform);

	bool doInitialize() override;

	void computeF(const SurgSim::Math::OdeState& state) override;

	void computeM(const SurgSim::Math::OdeState& state) override;

	void computeD(const SurgSim::Math::OdeState& state) override;

	void computeK(const SurgSim::Math::OdeState& state) override;

	void computeFMDK(const SurgSim::Math::OdeState& state) override;

private:
	/// Masses
	std::vector<std::shared_ptr<Mass>> m_masses;

	/// Springs
	std::vector<std::shared_ptr<Spring>> m_springs;

	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, M = mass, K = stiffness
	struct
	{
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;

	/// The Representation's asset as a MassSpring
	std::shared_ptr<MassSpring> m_mesh;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATION_H
