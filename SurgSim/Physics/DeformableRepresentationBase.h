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

#ifndef SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONBASE_H
#define SURGSIM_PHYSICS_DEFORMABLEREPRESENTATIONBASE_H

#include <string>

#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

class DeformableRepresentationState;

class DeformableRepresentationBase : public Representation
{
public:

	/// Constructor
	/// \param name Name of the representation.
	explicit DeformableRepresentationBase(const std::string& name) : Representation(name)
	{
	}

	/// Destructor
	virtual ~DeformableRepresentationBase()
	{
	}

	/// Set the initial state (in the global frame)
	/// \param initialState The initial state for this deformable representation (will potentially be changed)
	/// \note 'initialState' will be transformed by the initialPose, if any has been specified
	/// \note The parameter will be kept internally as the shared_ptr and the content will be transformed,
	/// \note   so after this call, do not expect 'initialState' to be unchanged.
	/// \note All internal states are initialized with the transformed initialState to make the simulation ready.
	/// \note This method also sets the number of dof for this Representation
	virtual void setInitialState(std::shared_ptr<DeformableRepresentationState> initialState) = 0;

	/// Get the current state (in the global frame), this is for use inside the physics calculation only
	/// \return The current state of this deformable representation
	virtual const std::shared_ptr<DeformableRepresentationState> getCurrentState() const = 0;

	/// Get the previous state (in the global frame), this is for use inside the physics calculation only
	/// \return The previous state of this deformable representation
	virtual const std::shared_ptr<DeformableRepresentationState> getPreviousState() const = 0;

	/// Get the final state (in the global frame), this is for use inside the physics calculation only
	/// \return The final state of this deformable representation
	virtual const std::shared_ptr<DeformableRepresentationState> getFinalState() const = 0;
};

}
}

#endif
