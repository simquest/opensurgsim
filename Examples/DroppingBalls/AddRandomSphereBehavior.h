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

#ifndef EXAMPLES_DROPPINGBALLS_ADDRANDOMSPHEREBEHAVIOR_H
#define EXAMPLES_DROPPINGBALLS_ADDRANDOMSPHEREBEHAVIOR_H

#include <random>

#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{

namespace Blocks
{

/// A behavior to add sphere elements into scene dynamically.
/// AddSphereBehavior will be processed by BehaviorManager in its update() call.
class AddRandomSphereBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	The name of add random sphere behavior.
	explicit AddRandomSphereBehavior(const std::string& name = "AddRandomSphereBehavior");

	/// Destructor
	~AddRandomSphereBehavior();

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	void update(double dt) override;

protected:
	/// Initialize the behavior
	bool doInitialize() override;
	/// Wakeup the behavior
	bool doWakeUp() override;

private:
	/// Control how often a sphere is added
	double m_totalTime;
	/// Record how many sphere have been added
	int m_numElements;
	/// Random number generator for randomized positions.
	std::default_random_engine m_generator;
	/// Distribution of random numbers for the x and z coordinates.
	std::uniform_real_distribution<double> m_distribution_xz;
	/// Distribution of random numbers for the y coordinate.
	std::uniform_real_distribution<double> m_distribution_y;
};

};  // namespace Blocks

};  // namespace SurgSim

#endif //EXAMPLES_DROPPINGBALLS_ADDRANDOMSPHEREBEHAVIOR_H
