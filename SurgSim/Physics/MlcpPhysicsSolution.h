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

#ifndef SURGSIM_PHYSICS_MLCPPHYSICSSOLUTION_H
#define SURGSIM_PHYSICS_MLCPPHYSICSSOLUTION_H

#include "SurgSim/Math/MlcpSolution.h"

namespace SurgSim
{
namespace Physics
{

/// The description of a solution to a \ref MlcpPhysicsProblem "physical MLCP problem".
///
/// The solution consists of a \ref SurgSim::Math::MlcpSolution "mathematical solution for the MLCP" and
/// the vector \ref dofCorrection containing the displacements (corrections) for each degree of freedom
/// present in the system.
///
/// \sa SurgSim::Math::MlcpSolution, MlcpPhysicsProblem, SurgSim::Math::MlcpSolver

struct MlcpPhysicsSolution: public SurgSim::Math::MlcpSolution
{
	/// Corrections to all of the degrees of freedom needed to satisfy the system equations.
	Vector dofCorrection;
};

};  // namespace Physics
};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_MLCPPHYSICSSOLUTION_H
