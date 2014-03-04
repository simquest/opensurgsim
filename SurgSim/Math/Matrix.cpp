// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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


#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{

namespace Math
{

template <>
void resizeMatrix<DiagonalMatrix>(DiagonalMatrix* A, unsigned int numRow, unsigned int numCol, bool zeroOut)
{
	SURGSIM_ASSERT(numRow == numCol) << "Trying to create a diagonal matrix non-square";
	if (A == nullptr)
	{
		return;
	}
	if (A->rows() != static_cast<int>(numRow) && A->cols() != static_cast<int>(numCol))
	{
		A->resize(static_cast<int>(numCol));
	}
	if (zeroOut)
	{
		A->setZero();
	}
}

};  // namespace Math

};  // namespace SurgSim
