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

/// \file
/// Utility function to test a LinearSpring

#ifndef SURGSIM_BLOCKS_UNITTESTS_SPRINGTESTUTILS_H
#define SURGSIM_BLOCKS_UNITTESTS_SPRINGTESTUTILS_H

#include <memory>

namespace SurgSim
{

namespace Math
{
class OdeState;
};

namespace Physics
{
class LinearSpring;
};

namespace Blocks
{

void springTest(std::shared_ptr<SurgSim::Physics::LinearSpring> spring,
	std::shared_ptr<SurgSim::Math::OdeState> state,
	size_t expectedNodeId0, size_t expectedNodeId1,
	double expectedStiffness, double expectedDamping);

}; // namespace Blocks

}; // namespace SurgSim

#endif // SURGSIM_BLOCKS_UNITTESTS_SPRINGTESTUTILS_H
