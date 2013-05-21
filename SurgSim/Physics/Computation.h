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

#ifndef SURGSIM_PHYSICS_TIMESTEP_H
#define SURGSIM_PHYSICS_TIMESTEP_H

#include <vector>

namespace SurgSim
{
namespace Physics
{

/// Encapsulates a calculation over a selection of objects, needs to be subclassed to be used
class Computation
{
public:

	/// Constructor
	Computation()
	{
	}
	/// Destructor
	virtual ~Computation()
	{
	}

	/// Public Interface execute this objects computations, dt is the time from
	/// the last update call in seconds
	inline void update(double dt)
	{
		doUpdate(dt);
	};

protected:

	/// Override this function to implement the computations specific behavior
	virtual void doUpdate(double dt) = 0;

};



}; // Physics
}; // SurgSim

#endif