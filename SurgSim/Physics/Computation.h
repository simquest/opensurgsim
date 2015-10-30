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

#ifndef SURGSIM_PHYSICS_COMPUTATION_H
#define SURGSIM_PHYSICS_COMPUTATION_H

#include <vector>
#include <memory>

#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace Physics
{

class PhysicsManagerState;

/// Encapsulates a calculation over a selection of objects, needs to be subclassed to be used
class Computation
{
public:

	/// Constructor
	/// \param doCopyState Specify if the output state in Computation::Update() is a copy or not of the input state
	explicit Computation(bool doCopyState);

	/// Destructor
	virtual ~Computation();

	/// Public Interface execute this objects computations, dt is the time from the last update
	/// call in seconds.
	/// \param	dt   	The time passed from the last update in seconds.
	/// \param	state	The physics state.
	/// \return	The changed state of the, depending on the setting of doCopyState this is either the same instance
	/// 		or a copied instance of the physics state.
	std::shared_ptr<PhysicsManagerState> update(double dt, const std::shared_ptr<PhysicsManagerState>& state);

	/// Sets up whether the computation will copy the state of PhysicsManagerState before executing.
	/// \param	val	Whether to create a copy of the PhysicsState before running the update fuction.
	void setDoCopyState(bool val);

	/// Query if this object is copying the PhysicsManagerState.
	/// \return	true if copying the state, false if not.
	bool isCopyingState();

	/// The class name for this class
	/// \note Use the SURGSIM_CLASSNAME macro in derived classes.
	/// \return The fully namespace qualified name of this class.
	virtual std::string getClassName() const = 0;

	/// Provides access to the update timer.
	/// \return The timer.
	Framework::Timer& getTimer();

protected:

	/// Override this function to implement the computations specific behavior
	virtual std::shared_ptr<PhysicsManagerState> doUpdate(
		const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state) = 0;

private:
	bool m_copyState;

	/// Copy the PhysicsManagerState object when isCopyingState() is true
	std::shared_ptr<PhysicsManagerState> preparePhysicsState(const std::shared_ptr<PhysicsManagerState>& state);

	/// The update timer.
	Framework::Timer m_timer;
};


}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_COMPUTATION_H
