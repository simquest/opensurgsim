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

#include "SurgSim/Math/MlcpProblem.h"

namespace SurgSim
{
namespace Math
{

MlcpProblem::~MlcpProblem()
{
}

void MlcpProblem::setZero(int numDof, int numConstraintDof, int numConstraints)
{
	A.resize(numConstraintDof, numConstraintDof);
	A.setZero();
	b.resize(numConstraintDof);
	b.setZero();
	mu.resize(numConstraints);
	mu.setZero();

	constraintTypes.clear();
}

MlcpProblem MlcpProblem::Zero(int numDof, int numConstraintDof, int numConstraints)
{
	MlcpProblem result;
	result.setZero(numDof, numConstraintDof, numConstraints);

	return result;

}

} // namespace SurgSim
} // namespace Math
