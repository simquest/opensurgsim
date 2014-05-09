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

#include "SurgSim/Collision/Location.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/RigidRepresentationState.h"

namespace SurgSim
{

namespace Collision
{
class Representation;
}

namespace Physics
{
class Localization;

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
	/// Reset the rigid representation state to its initial state
	void resetState();

	/// Get the initial state of the rigid representation
	/// \return The initial state (pose + lin/ang velocities)
	const RigidRepresentationState& getInitialState() const;
	/// Get the current state of the rigid representation
	/// \return The current state (pose + lin/ang velocities)
	const RigidRepresentationState& getCurrentState() const;
	/// Get the previous state of the rigid representation
	/// \return The previous state (pose + lin/ang velocities)
	const RigidRepresentationState& getPreviousState() const;

	std::shared_ptr<Localization> createLocalization(const SurgSim::Collision::Location& location);

	/// Set the initial parameters of the rigid representation
	/// \param parameters The initial parameters
	/// This will also set the current parameters to the initial parameters
	void setInitialParameters(const RigidRepresentationParameters& parameters);
	/// Set the current parameters of the rigid representation
	/// \param parameters The current parameters
	void setCurrentParameters(const RigidRepresentationParameters& parameters);

	/// Get the initial parameters of the rigid representation
	/// \return The initial parameters of the rigid representation
	const RigidRepresentationParameters& getInitialParameters() const;
	/// Get the current parameters of the rigid representation
	/// \return The current parameters of the rigid representation
	const RigidRepresentationParameters& getCurrentParameters() const;

	/// Set the collision representation for this physics representation, when the collision object
	/// is involved in a collision, the collision should be resolved inside the dynamics calculation.
	/// Specializes to register this representation in the collision representation if the collision representation
	/// is a RigidCollisionRepresentation.
	/// \param representation The collision representation to be used.
	virtual void setCollisionRepresentation(
		std::shared_ptr<SurgSim::Collision::Representation> representation) override;

	virtual	void beforeUpdate(double dt) override;
	virtual	void afterUpdate(double dt) override;

protected:
	bool doWakeUp() override;

	/// Initial rigid representation state (useful for reset)
	RigidRepresentationState m_initialState;
	/// Previous rigid representation state
	RigidRepresentationState m_previousState;
	/// Current rigid representation state
	RigidRepresentationState m_currentState;
	/// Last valid/final rigid representation state
	RigidRepresentationState m_finalState;

	/// Initial physical parameters
	RigidRepresentationParameters m_initialParameters;
	/// Current physical parameters
	RigidRepresentationParameters m_currentParameters;

	/// Creates typed localization.
	/// \tparam	T	Type of localization to create.
	/// \param	location	The location for the localization.
	/// \return	The new Localization;
	template <class T>
	std::shared_ptr<T> createTypedLocalization(const SurgSim::Collision::Location& location);

private:
	virtual void updateGlobalInertiaMatrices(const RigidRepresentationState& state) = 0;
};

}; // Physics

}; // SurgSim

#include "SurgSim/Physics/RigidRepresentationBase-inl.h"

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONBASE_H
