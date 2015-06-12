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

/// \file DeformableTestsUtility.h
/// Common functions for testing deformable compute/update functions.

#ifndef SURGSIM_PHYSICS_UNITTESTS_DEFORMABLETESTSUTILITY_H
#define SURGSIM_PHYSICS_UNITTESTS_DEFORMABLETESTSUTILITY_H

#include <memory>

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Physics
{

template <class T>
void testOdeEquationUpdate(std::shared_ptr<T> rep,
	const SurgSim::Math::OdeState& state, const SurgSim::Math::Vector& expectedF,
	const SurgSim::Math::Matrix& expectedM, const SurgSim::Math::Matrix& expectedD,
	const SurgSim::Math::Matrix& expectedK);

}
}

#include "SurgSim/Physics/UnitTests/DeformableTestsUtility-inl.h"

#endif // SURGSIM_PHYSICS_UNITTESTS_DEFORMABLETESTSUTILITY_H
