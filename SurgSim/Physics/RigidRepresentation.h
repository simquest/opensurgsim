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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATION_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATION_H

#include <SurgSim/Physics/RigidRepresentationBase.h>
#include <SurgSim/Physics/RigidRepresentationState.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

/// The RigidRepresentation class defines the dynamic rigid body representation
/// Note that the rigid representation is velocity-based, therefore its degrees of
/// freedom are the linear and angular velocities: 6 Dof
class RigidRepresentation : public RigidRepresentationBase
{
public:
	/// Constructor
	/// \param name The rigid representation's name
	explicit RigidRepresentation(const std::string& name);

	/// Destructor
	virtual ~RigidRepresentation();

	/// Set the initial state of the rigid representation
	/// \param state The initial state (pose + lin/ang velocities)
	/// This will also set the current/previous states to the initial state
	void setInitialState(const RigidRepresentationState& state)
	{
		m_initialState = state;
		m_currentState = state;
		m_previousState = state;

		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Set the initial parameters of the rigid representation
	/// \param parameters The initial parameters
	/// This will also set the current parameters to the initial parameters
	void setInitialParameters(const RigidRepresentationParameters& parameters)
	{
		m_initialParameters = parameters;
		m_currentParameters = parameters;

		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Set the current parameters of the rigid representation
	/// \param parameters The current parameters
	void setCurrentParameters(const RigidRepresentationParameters& parameters)
	{
		m_currentParameters = parameters;
		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Get the initial state of the rigid representation
	/// \return The initial state (pose + lin/ang velocities)
	const RigidRepresentationState& getInitialState() const
	{
		return m_initialState;
	}

	/// Get the initial parameters of the rigid representation
	/// \return The initial parameters of the rigid representation
	const RigidRepresentationParameters& getInitialParameters() const
	{
		return m_initialParameters;
	}

	/// Get the current state of the rigid representation
	/// \return The current state (pose + lin/ang velocities)
	const RigidRepresentationState& getCurrentState() const
	{
		return m_currentState;
	}

	/// Get the previous state of the rigid representation
	/// \return The previous state (pose + lin/ang velocities)
	const RigidRepresentationState& getPreviousState() const
	{
		return m_previousState;
	}

	/// Get the current parameters of the rigid representation
	/// \return The current parameters of the rigid representation
	const RigidRepresentationParameters& getCurrentParameters() const
	{
		return m_currentParameters;
	}

	/// Set the initial pose of the rigid representation
	/// \param pose The initial pose (translation + rotation)
	void setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
	{
		m_initialState.setPose(pose);
		m_currentState.setPose(pose);
		m_previousState.setPose(pose);

		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Get the initial pose of the rigid representation
	/// \return The initial pose (translation + rotation)
	const SurgSim::Math::RigidTransform3d& getInitialPose() const
	{
		return m_initialState.getPose();
	}

	/// Set the current pose of the rigid representation
	/// \param pose The current pose (translation + rotation)
	/// \note Does Not Apply to this representation (the pose is fully controlled by the
	/// physics simulation).
	void setPose(const SurgSim::Math::RigidTransform3d& pose)
	{
	}

	/// Get the previous pose of the rigid representation
	/// \return The previous pose (translation + rotation)
	const SurgSim::Math::RigidTransform3d& getPreviousPose() const
	{
		return m_previousState.getPose();
	}

	/// Get the current pose of the rigid representation
	/// \return The current pose (translation + rotation)
	const SurgSim::Math::RigidTransform3d& getCurrentPose() const
	{
		return m_currentState.getPose();
	}

	/// Get the final pose of the rigid representation
	/// \return The final pose (translation + rotation)
	const SurgSim::Math::RigidTransform3d& getPose() const
	{
		return m_finalState.getPose();
	}

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt);

	/// Update the representation state to the current time step (compute free motion)
	/// \param dt The time step (in seconds)
	void update(double dt);

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	void afterUpdate(double dt);

	/// Reset the rigid representation state to its initial state
	void resetState()
	{
		Representation::resetState();

		m_currentState  = m_initialState;
		m_previousState = m_initialState;
		m_finalState    = m_initialState;

		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Reset the rigid representation parameters to the initial parameters
	void resetParameters()
	{
		Representation::resetParameters();

		m_currentParameters = m_initialParameters;

		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Retrieve the rigid body 6x6 compliance matrix
	/// \return the 6x6 compliance matrix
	const Eigen::Matrix<double, 6,6, Eigen::DontAlign | Eigen::RowMajor>& getComplianceMatrix() const
	{
		return m_C;
	}

protected:
	/// Inertia matrices in global coordinates
	SurgSim::Math::Matrix33d m_globalInertia;

	/// Inverse of inertia matrix in global coordinates
	SurgSim::Math::Matrix33d m_invGlobalInertia;

	/// Current force applied on the rigid representation (in N)
	SurgSim::Math::Vector3d m_force;
	/// Current torque applied on the rigid representation (in N.m)
	SurgSim::Math::Vector3d m_torque;

	/// Compliance matrix (size of the number of Dof = 6)
	Eigen::Matrix<double, 6,6, Eigen::DontAlign | Eigen::RowMajor> m_C;

private:
	/// Compute compliance matrix (internal data structure)
	/// \param dt The time step in use
	void computeComplianceMatrix(double dt);

	/// Update global inertia matrices (internal data structure)
	/// \param state The state of the rigid representation to use for the update
	void updateGlobalInertiaMatrices(const RigidRepresentationState& state);

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
};

}; // Physics

}; // SurgSim

#endif /// SURGSIM_PHYSICS_RIGIDREPRESENTATION_H
