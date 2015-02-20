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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_H

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentationBaseLocalization.h"
#include "SurgSim/Physics/RigidRepresentationState.h"

namespace SurgSim
{

namespace Collision
{
class Representation;
}

namespace Physics
{

/// The RigidRepresentationBase class defines the base class for
/// all rigid motion based representations (fixed, rigid body, rigid body + vtc,...)
class RigidRepresentationBase : public Representation
{
public:
	/// Constructor
	/// \param name The rigid representation's name
	explicit RigidRepresentationBase(const std::string& name);

	/// Destructor
	virtual ~RigidRepresentationBase();

	/// Set the initial state of the rigid representation
	/// \param state The initial state (pose + lin/ang velocities)
	/// This will also set the current/previous states to the initial state
	void setInitialState(const RigidRepresentationState& state);

	void resetState() override;

	/// Get the initial state of the rigid representation
	/// \return The initial state (pose + lin/ang velocities)
	const RigidRepresentationState& getInitialState() const;
	/// Get the current state of the rigid representation
	/// \return The current state (pose + lin/ang velocities)
	const RigidRepresentationState& getCurrentState() const;
	/// Get the previous state of the rigid representation
	/// \return The previous state (pose + lin/ang velocities)
	const RigidRepresentationState& getPreviousState() const;

	std::shared_ptr<Localization> createLocalization(const SurgSim::DataStructures::Location& location) override;

	/// Set the mass density of the rigid representation
	/// \param rho The density (in Kg.m-3)
	void setDensity(double rho);

	/// Get the mass density of the rigid representation
	/// \return The density if it has been provided, 0 otherwise (in Kg.m-3)
	double getDensity() const;

	/// Get the mass of the rigid body
	/// \return The mass (in Kg)
	double getMass() const;

	/// Get the mass center of the rigid body
	/// \return The mass center (in local coordinate)
	const SurgSim::Math::Vector3d& getMassCenter() const;

	/// Get the local inertia 3x3 matrix of the rigid body
	/// \return The inertia 3x3 matrix of the object
	const SurgSim::Math::Matrix33d& getLocalInertia() const;

	/// Set the linear damping parameter
	/// \param linearDamping The linear damping parameter (in N.s.m-1)
	void setLinearDamping(double linearDamping);

	/// Get the linear damping parameter
	/// \return The linear damping parameter (in N.s.m-1)
	double getLinearDamping() const;

	/// Set the angular damping parameter
	/// \param angularDamping The angular damping parameter (in N.m.s.rad-1)
	void setAngularDamping(double angularDamping);

	/// Get the angular damping parameter
	/// \return The angular damping parameter (in N.m.s.rad-1)
	double getAngularDamping() const;

	/// Set the shape to use internally for physical parameters computation
	/// \param shape The shape to use for the mass/inertia calculation
	/// \note Also add the shape to the shape list if it has not been added yet
	void setShape(const std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Get the shape used internally for physical parameters computation
	/// \return The shape used for calculation, nullptr if none exist
	const std::shared_ptr<SurgSim::Math::Shape> getShape() const;

	/// Set the collision representation for this physics representation, when the collision object
	/// is involved in a collision, the collision should be resolved inside the dynamics calculation.
	/// Specializes to register this representation in the collision representation if the collision representation
	/// is a RigidCollisionRepresentation.
	/// \param representation The collision representation to be used.
	void setCollisionRepresentation(std::shared_ptr<SurgSim::Collision::Representation> representation) override;

	void beforeUpdate(double dt) override;
	void afterUpdate(double dt) override;

protected:
	bool doInitialize() override;
	bool doWakeUp() override;

	/// Initial rigid representation state (useful for reset)
	RigidRepresentationState m_initialState;
	/// Previous rigid representation state
	RigidRepresentationState m_previousState;
	/// Current rigid representation state
	RigidRepresentationState m_currentState;
	/// Last valid/final rigid representation state
	RigidRepresentationState m_finalState;

	/// Validity of the parameters
	bool m_parametersValid;

	/// Density of the object (in Kg.m-3)
	double m_rho;

	/// Total mass of the object (in Kg)
	double m_mass;

	/// Linear damping parameter (in N.s.m-1 or Kg.s-1)
	double m_linearDamping;

	/// Angular damping parameter (in N.m.s.rad-1)
	double m_angularDamping;

	/// Mass-center of the object
	SurgSim::Math::Vector3d m_massCenter;

	/// Inertia matrix in local coordinates
	SurgSim::Math::Matrix33d m_localInertia;

	/// Shape to be used for the mass/inertia calculation
	std::shared_ptr<SurgSim::Math::Shape> m_shape;

	/// Creates typed localization.
	/// \tparam	T	Type of localization to create.
	/// \param	location	The location for the localization.
	/// \return	The new Localization;
	template <class T>
	std::shared_ptr<T> createTypedLocalization(const SurgSim::DataStructures::Location& location);

private:
	/// Updates mass, mass center and inertia when density and/or shape used for mass inertia is updated.
	void updateProperties();

	virtual void updateGlobalInertiaMatrices(const RigidRepresentationState& state) = 0;
};

}; // Physics
}; // SurgSim

#include "SurgSim/Physics/RigidRepresentationBase-inl.h"

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_H
