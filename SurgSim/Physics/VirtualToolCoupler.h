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

#include <SurgSim/Physics/RigidRepresentationState.h>
#include <SurgSim/Framework/Behavior.h>

namespace SurgSim
{

namespace Input
{
	class InputComponent;
}

namespace Physics
{

class RigidRepresentation;

/// The VirtualToolCoupler couples a rigid object to an input through a spring.
class VirtualToolCoupler : public SurgSim::Framework::Behavior
{
public:
    /// Constructor
    /// \param    name    Name of the behavior
    /// \param    from    Input to get the pose
    /// \param    to      Rigid Representation to control
    /// \param    poseName Name of the pose data in the input to transfer
    VirtualToolCoupler(const std::string& name, std::shared_ptr<SurgSim::Input::InputComponent> input,
				std::shared_ptr<SurgSim::Physics::RigidRepresentation> rigid, const std::string& poseName = "pose");

	~VirtualToolCoupler();

    /// Update the behavior
    /// \param dt    The length of time (seconds) between update calls.
    virtual void update(double dt);

	/// Set vtc linear stiffness
	/// \param linearStiffness The stiffness of the vtc in linear mode (in N·m-1)
	void setLinearStiffness(double linearStiffness);

	/// Set vtc linear damping
	/// \param linearDamping The damping of the vtc in linear mode (in N·s·m-1 or Kg·s-1)
	void setLinearDamping(double linearDamping);

	/// Set vtc angular stiffness
	/// \param angularStiffness The stiffness of the vtc in angular mode (in N·m rad-1)
	void setAngularStiffness(double angularStiffness);

	/// Set vtc angular damping
	/// \param angularDamping The damping of the vtc in angular mode (in N·m·s·rad-1)
	void setAngularDamping(double angularDamping);

protected:
    /// Initialize the behavior
    virtual bool doInitialize();

    /// Wakeup the behavior
    virtual bool doWakeUp();

	virtual int getTargetManagerType() const;

private:
    std::shared_ptr<SurgSim::Input::InputComponent> m_input;
    std::shared_ptr<SurgSim::Physics::RigidRepresentation> m_rigid;
    std::string m_poseName;
	RigidRepresentationState m_previousState;

	/// Vtc stiffness parameter in linear mode (in N·m-1)
	double m_linearStiffness;

	/// Vtc damping parameter in linear mode (in N·s·m-1 or Kg·s-1)
	double m_linearDamping;

	/// Vtc stiffness parameter in angular mode (in N·m rad-1)
	double m_angularStiffness;

	/// Vtc damping parameter in angular mode (in N·m·s·rad-1)
	double m_angularDamping;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VIRTUALTOOLCOUPLER_H
