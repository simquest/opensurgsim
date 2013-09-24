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

#ifndef SURGSIM_PHYSICS_VTCBEHAVIOR_H
#define SURGSIM_PHYSICS_VTCBEHAVIOR_H

#include <SurgSim/Physics/VtcParameters.h>
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

/// The VtcBehavior controls a rigid body representation using a  
/// Virtual tool coupler (i.e. god-object or proxy)
class VtcBehavior : public SurgSim::Framework::Behavior
{
public:
    /// Constructor
    /// \param    name    Name of the behavior
    /// \param    from    Input to get the pose
    /// \param    to      Rigid Representation to control
    /// \param    poseName Name of the pose data in the input to transfer
    VtcBehavior(const std::string& name, std::shared_ptr<SurgSim::Input::InputComponent> input,
				std::shared_ptr<SurgSim::Physics::RigidRepresentation> rigid, const std::string& poseName = "pose");

	~VtcBehavior();

	/// Set the current Vtc parameters
	/// \param parameters The current Vtc parameters
	void setParameters(const VtcParameters& parameters);

	/// Get the current Vtc parameters
	/// \return The current Vtc parameters
	const VtcParameters& getParameters() const;

    /// Update the behavior
    /// \param dt    The length of time (seconds) between update calls.
    virtual void update(double dt);


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
	VtcParameters m_parameters;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VTCRIGIDREPRESENTATION_H
