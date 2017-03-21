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

#ifndef SURGSIM_PHYSICS_VIRTUALTOOLCOUPLER_H
#define SURGSIM_PHYSICS_VIRTUALTOOLCOUPLER_H

#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{

namespace Input
{
class InputComponent;
class OutputComponent;
}

namespace Physics
{

class RigidRepresentation;

SURGSIM_STATIC_REGISTRATION(VirtualToolCoupler);

/// The VirtualToolCoupler couples a rigid object to an input/output device through a spring and damper.
/// The object will follow the pose provided by the device. If an Output is connected, it is provided
/// forces and torques that will push the device towards matching the object's pose and velocity.
/// This "virtual coupling" or "god-object" paradigm is common for haptic applications utilizing a device that
/// may update significantly faster than the physics computation thread.
///
/// For an overview of haptics see:
///		Salisbury, Kenneth, Francois Conti, and Federico Barbagli. "Haptic rendering: introductory concepts."
///		Computer Graphics and Applications, IEEE 24.2 (2004): 24-32.
///
/// For an introduction to virtual coupling see:
///		Colgate, J. Edward, Michael C. Stanley, and J. Michael Brown. "Issues in the haptic display of tool use."
///		Intelligent Robots and Systems 95.'Human Robot Interaction and Cooperative Robots',
///		Proceedings. 1995 IEEE/RSJ International Conference on. Vol. 3. IEEE, 1995.
class VirtualToolCoupler : public Framework::Behavior
{
public:
	/// Constructor
	/// \param name Name of the behavior
	explicit VirtualToolCoupler(const std::string& name);

	~VirtualToolCoupler();

	SURGSIM_CLASSNAME(SurgSim::Physics::VirtualToolCoupler);

	/// \return Input Component to get the pose from
	const std::shared_ptr<Framework::Component> getInput();

	/// Set the Input Component.
	/// The force calculations rely upon pose and velocity.  If the input DataGroup does not contain a pose, no forces
	/// will be calculated.  If the input DataGroup does not contain linear velocity or angular velocity, they will
	/// be estimated.
	/// \param input Input Component to get the pose from
	void setInput(const std::shared_ptr<Framework::Component> input);

	/// \return Output Component to send forces and torques
	const std::shared_ptr<Framework::Component> getOutput();

	/// Set the Output Component (if any)
	/// \param output Output Component to send forces and torques
	void setOutput(const std::shared_ptr<Framework::Component> output);

	/// \return Rigid Representation that provides state and receives external forces and torques
	const std::shared_ptr<Framework::Component> getRepresentation();

	/// Set the Physics Representation which follows the input
	/// \param rigid Rigid Representation that provides state and receives external forces and torques
	void setRepresentation(const std::shared_ptr<Framework::Component> rigid);

	/// Get whether or not the haptic forces should be provided only during collisions.
	/// \return false if the VTC forces and torques are sent to the output device (if any) at all times.  true if
	///		zeros are sent for the forces and torques unless the tool is colliding.
	bool isHapticOutputOnlyWhenColliding() const;

	/// Set whether or not the haptic forces should be provided only during collisions.
	/// \param haptic false to send the VTC forces and torques to the output device (if any) at all times.  true to
	///		send zeros for the forces and torques unless the tool is colliding.
	void setHapticOutputOnlyWhenColliding(bool haptic);

	/// \return Name of the pose data in the input to transfer
	const std::string& getPoseName();

	/// Set the name of the pose entry in the input DataGroup
	/// \param    poseName Name of the pose data in the input to transfer
	void setPoseName(const std::string& poseName = DataStructures::Names::POSE);

	void update(double dt) override;

	/// Override the linear stiffness connecting the input device and the physics representation
	/// If this value is not provided, the stiffness will be automatically tuned using
	/// the properties of the Representation
	/// \param linearStiffness The stiffness of the vtc in linear mode (in N·m-1)
	void overrideLinearStiffness(double linearStiffness);

	/// \return The stiffness of the vtc in linear mode (in N·m-1)
	double getLinearStiffness();

	/// Override the linear damping connecting the input device and the physics representation
	/// If this value is not provided, the damping will be automatically tuned using
	/// the properties of the Representation
	/// \param linearDamping The damping of the vtc in linear mode (in N·s·m-1 or Kg·s-1)
	void overrideLinearDamping(double linearDamping);

	/// \return The damping of the vtc in linear mode (in N·s·m-1 or Kg·s-1)
	double getLinearDamping();

	/// Override the angular stiffness connecting the input device and the physics representation
	/// If this value is not provided, the stiffness will be automatically tuned using
	/// the properties of the Representation
	/// \param angularStiffness The stiffness of the vtc in angular mode (in N·m rad-1)
	void overrideAngularStiffness(double angularStiffness);

	/// \return The stiffness of the vtc in angular mode (in N·m rad-1)
	double getAngularStiffness();

	/// Override the angular damping connecting the input device and the physics representation
	/// If this value is not provided, the damping will be automatically tuned using
	/// the properties of the Representation
	/// \param angularDamping The damping of the vtc in angular mode (in N·m·s·rad-1)
	void overrideAngularDamping(double angularDamping);

	/// \return The damping of the vtc in angular mode (in N·m·s·rad-1)
	double getAngularDamping();

	/// Override the point of attachment to the Representation
	/// If this value is not provided, the point of attachment will be automatically
	/// set to the Representation's center of mass.
	/// \param attachment The attachment point in the Representations local coordinate frame
	void overrideAttachmentPoint(const Math::Vector3d& attachment);

	/// Get the point of attachment on the Representation
	/// \return The attachment point in the Representations local coordinate frame
	const Math::Vector3d& getAttachmentPoint();

	/// Enable/disable torques that simulate inertia.  This setting only has an effect if the attachment point is not
	/// the mass center.
	/// \sa overrideAttachmentPoint
	/// \param calculateInertialTorques true to simulate inertia.
	void setCalculateInertialTorques(bool calculateInertialTorques);

	/// Get whether the calculated torques will simulate inertia.  This setting only has an effect if the attachment
	/// point is not the mass center.
	/// \sa overrideAttachmentPoint
	/// \return true if inertia is being simulated.
	bool getCalculateInertialTorques() const;

	void doRetire() override;

	void setLocalActive(bool val) override;

protected:
	bool doInitialize() override;
	bool doWakeUp() override;
	int getTargetManagerType() const override;

	/// \return The DataGroup to be sent to the device via the OutputComponent.
	virtual DataStructures::DataGroup buildOutputData();

	/// Used for Serialization.
	/// \param linearStiffness The OptionalValue object containing the stiffness of the vtc in linear mode (in N·m-1)
	void setOptionalLinearStiffness(const DataStructures::OptionalValue<double>& linearStiffness);

	/// Used for Serialization.
	/// \return The OptionalValue object containing the stiffness of the vtc in linear mode (in N·m-1)
	const DataStructures::OptionalValue<double>& getOptionalLinearStiffness() const;

	/// Used for Serialization.
	/// \param linearDamping The OptionalValue object containing the damping of the vtc in linear
	/// mode (in N·s·m-1 or Kg·s-1)
	void setOptionalLinearDamping(const DataStructures::OptionalValue<double>& linearDamping);

	/// Used for Serialization.
	/// \return The OptionalValue object containing the damping of the vtc in linear mode (in N·s·m-1 or Kg·s-1)
	const DataStructures::OptionalValue<double>& getOptionalLinearDamping() const;

	/// Used for Serialization.
	/// \param angularStiffness The OptionalValue object containing the stiffness of the vtc in angular
	/// mode (in N·m rad-1)
	void setOptionalAngularStiffness(const DataStructures::OptionalValue<double>& angularStiffness);

	/// Used for Serialization.
	/// \return The OptionalValue object containing the stiffness of the vtc in angular mode (in N·m rad-1)
	const DataStructures::OptionalValue<double>& getOptionalAngularStiffness() const;

	/// Used for Serialization.
	/// \param angularDamping The OptionalValue object containing the damping of the vtc in angular
	/// mode (in N·m·s·rad-1)
	void setOptionalAngularDamping(const DataStructures::OptionalValue<double>& angularDamping);

	/// Used for Serialization.
	/// \return The OptionalValue object containing the damping of the vtc in angular mode (in N·m·s·rad-1)
	const DataStructures::OptionalValue<double>& getOptionalAngularDamping() const;

	/// Used for Serialization.
	/// \param attachmentPoint The OptionalValue object containing the attachment point.
	void setOptionalAttachmentPoint(const DataStructures::OptionalValue<Math::Vector3d>& attachmentPoint);

	/// Used for Serialization.
	/// \return The OptionalValue object containing the attachment point.
	const DataStructures::OptionalValue<Math::Vector3d>& getOptionalAttachmentPoint() const;

	/// User supplied Vtc stiffness parameter in linear mode (in N·m-1)
	DataStructures::OptionalValue<double> m_optionalLinearStiffness;

	/// User supplied Vtc damping parameter in linear mode (in N·s·m-1 or Kg·s-1)
	DataStructures::OptionalValue<double> m_optionalLinearDamping;

	/// User supplied Vtc stiffness parameter in angular mode (in N·m rad-1)
	DataStructures::OptionalValue<double> m_optionalAngularStiffness;

	/// User supplied Vtc damping parameter in angular mode (in N·m·s·rad-1)
	DataStructures::OptionalValue<double> m_optionalAngularDamping;

	/// User supplied attachment point
	DataStructures::OptionalValue<Math::Vector3d> m_optionalAttachmentPoint;

	/// The DataGroup to output
	DataStructures::DataGroup m_outputData;

	/// The input component.
	std::shared_ptr<Input::InputComponent> m_input;

	/// The output component.
	std::shared_ptr<Input::OutputComponent> m_output;

private:
	std::shared_ptr<RigidRepresentation> m_rigid;
	std::string m_poseName;

	/// Used Vtc stiffness parameter in linear mode (in N·m-1)
	double m_linearStiffness;

	/// Used Vtc damping parameter in linear mode (in N·s·m-1 or Kg·s-1)
	double m_linearDamping;

	/// Used Vtc stiffness parameter in angular mode (in N·m rad-1)
	double m_angularStiffness;

	/// Used Vtc damping parameter in angular mode (in N·m·s·rad-1)
	double m_angularDamping;

	/// Scaling factor for the forces sent to the OutputComponent
	double m_outputForceScaling;

	/// Scaling factor for the torques sent to the OutputComponent
	double m_outputTorqueScaling;

	/// The input's point of attachment in the local frame, i.e., the same frame in which the mass center is defined.
	Math::Vector3d m_localAttachmentPoint;

	/// Whether or not the calculated torques will simulate inertia.  This setting only has an effect if the device
	/// input point is not the mass center.
	bool m_calculateInertialTorques;

	/// The logger.
	std::shared_ptr<Framework::Logger> m_logger;

	/// Whether or not the VTC sends forces and torques to the output device (if any) only when the tool is colliding.
	bool m_hapticOutputOnlyWhenColliding;

	/// The previous input pose.
	Math::UnalignedRigidTransform3d m_previousInputPose;

	///@{
	/// Cached DataGroup indices.
	int m_poseIndex;
	int m_linearVelocityIndex;
	int m_angularVelocityIndex;
	int m_forceIndex;
	int m_torqueIndex;
	int m_inputLinearVelocityIndex;
	int m_inputAngularVelocityIndex;
	int m_inputPoseIndex;
	int m_springJacobianIndex;
	int m_damperJacobianIndex;
	///@}
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VIRTUALTOOLCOUPLER_H
