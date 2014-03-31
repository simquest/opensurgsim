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
#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Framework/Behavior.h"

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

/// The VirtualToolCoupler couples a rigid object to an input/output device through a spring and damper.  If the device
/// will output forces and/or torques, we pass it a force (and/or torque) as well as the derivatives (Jacobians) of
/// the force with respect to position and velocity, so that the device can recalculate its forces at its update rate.
class VirtualToolCoupler : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param name Name of the behavior
	explicit VirtualToolCoupler(const std::string& name);

	~VirtualToolCoupler();

	/// Set the Input Component
	/// \param input Input Component to get the pose from
	void setInput(const std::shared_ptr<SurgSim::Input::InputComponent> input);

	/// Set the Output Component (if any)
	/// \param output Output Component to send forces and torques
	void setOutput(const std::shared_ptr<SurgSim::Input::OutputComponent> output);

	/// Set the Physics Representation which follows the input
	/// \param rigid Rigid Representation that provides state and receives external forces and torques
	void setRepresentation(const std::shared_ptr<SurgSim::Physics::RigidRepresentation> rigid);

	/// Set the name of the pose entry in the input DataGroup
	/// \param    poseName Name of the pose data in the input to transfer
	void setPoseName(const std::string& poseName = "pose");

	virtual void update(double dt) override;

	/// Set linear stiffness connecting the input device and the physics representation
	/// If this value is not provided, the stiffness will be automatically tuned using
	/// the properties of the Representation
	/// \param linearStiffness The stiffness of the vtc in linear mode (in N·m-1)
	void setLinearStiffness(double linearStiffness);

	/// Set linear damping connecting the input device and the physics representation
	/// If this value is not provided, the damping will be automatically tuned using
	/// the properties of the Representation
	/// \param linearDamping The damping of the vtc in linear mode (in N·s·m-1 or Kg·s-1)
	void setLinearDamping(double linearDamping);

	/// Set angular stiffness connecting the input device and the physics representation
	/// If this value is not provided, the stiffness will be automatically tuned using
	/// the properties of the Representation
	/// \param angularStiffness The stiffness of the vtc in angular mode (in N·m rad-1)
	void setAngularStiffness(double angularStiffness);

	/// Set angular damping connecting the input device and the physics representation
	/// If this value is not provided, the damping will be automatically tuned using
	/// the properties of the Representation
	/// \param angularDamping The damping of the vtc in angular mode (in N·m·s·rad-1)
	void setAngularDamping(double angularDamping);

	/// Set the scaling term for the force sent to the output component.
	/// \param forceScaling The factor to multiply the forces.
	void setOutputForceScaling(double forceScaling);

	/// Set the scaling term for the torque sent to the output component.
	/// \param torqueScaling The factor to multiply the torque.
	void setOutputTorqueScaling(double torqueScaling);

protected:
	virtual bool doInitialize() override;
	virtual bool doWakeUp() override;
	virtual int getTargetManagerType() const override;

	/// Vtc stiffness parameter in linear mode (in N·m-1)
	SurgSim::DataStructures::OptionalValue<double> m_linearStiffness;

	/// Vtc damping parameter in linear mode (in N·s·m-1 or Kg·s-1)
	SurgSim::DataStructures::OptionalValue<double> m_linearDamping;

	/// Vtc stiffness parameter in angular mode (in N·m rad-1)
	SurgSim::DataStructures::OptionalValue<double> m_angularStiffness;

	/// Vtc damping parameter in angular mode (in N·m·s·rad-1)
	SurgSim::DataStructures::OptionalValue<double> m_angularDamping;

private:
	std::shared_ptr<SurgSim::Input::InputComponent> m_input;
	std::shared_ptr<SurgSim::Input::OutputComponent> m_output;
	std::shared_ptr<SurgSim::Physics::RigidRepresentation> m_rigid;
	std::string m_poseName;

	/// Scaling factor for the forces sent to the OutputComponent
	double m_outputForceScaling;

	/// Scaling factor for the torques sent to the OutputComponent
	double m_outputTorqueScaling;

	/// The DataGroup to output
	SurgSim::DataStructures::DataGroup m_outputData;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VIRTUALTOOLCOUPLER_H
