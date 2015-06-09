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

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{

namespace Math
{

const std::shared_ptr<OdeState> OdeEquation::getInitialState() const
{
	return m_initialState;
}

void OdeEquation::update(const OdeState& state, bool updateF, bool updateM, bool updateD, bool updateK)
{
	SURGSIM_LOG_DEBUG(SurgSim::Framework::Logger::getLogger("Math/OdeEquation"))
		<< "OdeEquation::update() has been called. One of the sub-classes has not overriden it.";
}

}; // namespace Math

}; // namespace SurgSim
