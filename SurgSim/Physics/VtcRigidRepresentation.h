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

#ifndef SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_H
#define SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_H

#include <SurgSim/Physics/RigidRepresentationBase.h>
#include <SurgSim/Physics/RigidRepresentationState.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>
#include <SurgSim/Physics/VtcRigidParameters.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

/// The VtcRigidRepresentation class defines a rigid body representation associated with a
/// Virtual tool coupler (i.e. god-object or proxy)
/// Note that the rigid representation is velocity-based, therefore its degrees of
/// freedom are the linear and angular velocities: 6 Dof
/// \note The physical rigid body is driven by the Vtc through setPose(...)
/// \note setPose sets the proxy (Vtc) pose.
/// \note getPose gets the virtual rigid body pose.
class VtcRigidRepresentation : public RigidRepresentationBase
{
public:
	/// Constructor
	/// \param name The rigid representation's name
	explicit VtcRigidRepresentation(const std::string& name);

	/// Destructor
	virtual ~VtcRigidRepresentation();

	virtual RepresentationType getType() const override;

	/// Set the current pose of the rigid representation
	/// \param pose The current pose (translation + rotation)
	/// \note This is done through the Vtc proxy !
	/// \note We let the end-user drive the Vtc, not the virtual rigid representation directly
	void setPose(const SurgSim::Math::RigidTransform3d& pose)
	{
		m_currentVtcState.setPose(pose);
	}


	/// Set the initial parameters of the rigid representation
	/// \param parameters The initial parameters
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

	/// Get the initial parameters of the rigid representation
	/// \return The initial parameters of the rigid representation
	const RigidRepresentationParameters& getInitialParameters() const
	{
		return m_initialParameters;
	}


	/// Get the current parameters of the rigid representation
	/// \return The current parameters of the rigid representation
	const RigidRepresentationParameters& getCurrentParameters() const
	{
		return m_currentParameters;
	}

	/// Set the initial Vtc proxy state
	/// \param state The initial Vtc state (pose + lin/ang velocities)
	void setInitialVtcState(const RigidRepresentationState& state)
	{
		m_initialVtcState = state;
		m_currentVtcState = state;
		m_previousVtcState = state;
	}

	/// Set the initial Vtc parameters
	/// \param parameters The initial Vtc parameters
	void setInitialVtcParameters(const VtcRigidParameters& parameters)
	{
		m_initialVtcParameters = parameters;
		m_currentVtcParameters = parameters;
	}

	/// Set the current Vtc parameters
	/// \param parameters The current Vtc parameters
	void setCurrentVtcParameters(const VtcRigidParameters& parameters)
	{
		m_currentVtcParameters = parameters;
	}

	/// Get the initial Vtc state
	/// \return The initial Vtc state (pose + lin/ang velocities)
	const RigidRepresentationState& getInitialVtcState() const
	{
		return m_initialVtcState;
	}

	/// Get the initial Vtc parameters
	/// \return The initial Vtc parameters
	const VtcRigidParameters& getInitialVtcParameters() const
	{
		return m_initialVtcParameters;
	}

	/// Get the current Vtc state
	/// \return The current Vtc state (pose + lin/ang velocities)
	const RigidRepresentationState& getCurrentVtcState() const
	{
		return m_currentVtcState;
	}

	/// Get the previous Vtc state
	/// \return The previous Vtc state (pose + lin/ang velocities)
	const RigidRepresentationState& getPreviousVtcState() const
	{
		return m_previousVtcState;
	}

	/// Get the current Vtc parameters
	/// \return The current Vtc parameters
	const VtcRigidParameters& getCurrentVtcParameters() const
	{
		return m_currentVtcParameters;
	}

	/// Get the final pose of the rigid representation
	/// \return The final pose (translation + rotation)
	/// \note The end-user set the pose of the Vtc but retrieve information from the virtual rigid representation
	const SurgSim::Math::RigidTransform3d& getPose() const
	{
		return m_finalState.getPose();
	}

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt);

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	void update(double dt);

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	void afterUpdate(double dt);

	/// Reset the rigid representation parameters to their initial values
	/// \note Does not reset the Vtc parameters
	void resetParameters()
	{
		RigidRepresentationBase::resetParameters();

		m_currentParameters = m_initialParameters;

		updateGlobalInertiaMatrices(m_currentState);
	}

	/// Reset the Vtc parameters to their initial values
	void resetVtcParameters()
	{
		m_currentVtcParameters = m_initialVtcParameters;
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



	/// Initial physical parameters
	RigidRepresentationParameters m_initialParameters;

	/// Current physical parameters
	RigidRepresentationParameters m_currentParameters;

	/// Initial Vtc state (useful for reset)
	RigidRepresentationState m_initialVtcState;

	/// Previous Vtc state
	RigidRepresentationState m_previousVtcState;

	/// Current Vtc state
	RigidRepresentationState m_currentVtcState;

	/// Initial Vtc parameters
	VtcRigidParameters m_initialVtcParameters;

	/// Current Vtc parameters
	VtcRigidParameters m_currentVtcParameters;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_H
