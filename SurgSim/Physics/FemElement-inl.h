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

#ifndef SURGSIM_PHYSICS_FEMELEMENT_INL_H
#define SURGSIM_PHYSICS_FEMELEMENT_INL_H

#include <vector>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/SparseMatrix.h"

namespace SurgSim
{

namespace Physics
{

template <typename DerivedSub, typename T, int Opt, typename Index>
void FemElement::assembleMatrixBlocks(const DerivedSub& subMatrix, const std::vector<size_t> blockIds,
									  Index blockSize, Eigen::SparseMatrix<T, Opt, Index>* matrix,
									  bool initialize)
{
	using SurgSim::Math::addSubMatrix;

	const Index numBlocks = static_cast<Index>(blockIds.size());
	for (Index block0 = 0; block0 < numBlocks; block0++)
	{
		Index subRow = blockSize * block0;
		for (Index block1 = 0; block1 < numBlocks; block1++)
		{
			Index subCol = blockSize * block1;
			addSubMatrix(subMatrix.block(subRow, subCol, blockSize, blockSize),
				static_cast<Index>(blockIds[block0]), static_cast<Index>(blockIds[block1]), matrix, initialize);
		}
	}
}

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMELEMENT_INL_H
