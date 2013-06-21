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

#ifndef SURGSIM_PHYSICS_FIXEDREPRESENTATION_H
#define SURGSIM_PHYSICS_FIXEDREPRESENTATION_H

#include <SurgSim/Physics/RigidRepresentationBase.h>
#include <SurgSim/Physics/RigidRepresentationBaseState.h>

namespace SurgSim
{

namespace Physics
{

/// The FixedRepresentation class represents a physics entity without any motion nor
/// compliance against which others physics entities can interact
class FixedRepresentation : public RigidRepresentationBase
{
public:
	/// Constructor
	/// \param name The fixed representation's name
	explicit FixedRepresentation(const std::string& name)
		: RigidRepresentationBase(name)
	{
	}

	/// Destructor
	virtual ~FixedRepresentation()
	{
	}

	/// Set the current pose of the rigid representation
	/// \param pose The current pose (translation + rotation)
	void setPose(const RigidTransform3d& pose)
	{
		m_previousState = m_currentState;
		m_finalState.setPose(pose);
		m_currentState.setPose(pose);
	}

	virtual void updateGlobalInertiaMatrices(const RigidRepresentationState& state) 
	{
		// Do Nothing it is a fixed object
	}

private:

};

}; // Physics

}; // SurgSim

#endif /// SURGSIM_PHYSICS_FIXEDREPRESENTATION_H
