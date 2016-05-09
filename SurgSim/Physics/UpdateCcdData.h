// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_UPDATECCDDATA_H
#define SURGSIM_PHYSICS_UPDATECCDDATA_H

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Physics/Computation.h"

namespace SurgSim
{

namespace Physics
{

class UpdateCcdData : public Computation
{
public:
	/// Constructor
	explicit UpdateCcdData(bool copyState);

	SURGSIM_CLASSNAME(SurgSim::Physics::UpdateCcdData);

	/// Destructor
	~UpdateCcdData();

	/// Change of behavior, the value transported is not the complete 'interval' but is the percentage
	/// of the last interval where the previous impact was found, this is to trigger the
	/// correct interpolation of the state from the previous state to the interpolated state. This
	/// value is relative to the last interval to match the behavior of the rest of the system
	/// \param interval Parameter to be used to interpolate the previousPhysicsState
	/// \param state normal PhysicsManagerState
	std::shared_ptr<PhysicsManagerState>
	doUpdate(const double& interval, const std::shared_ptr<PhysicsManagerState>& state) override;

private:

};

}
}

#endif
