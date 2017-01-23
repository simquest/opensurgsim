// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PARALLEL_LINEARSOLVEANDINVERSEOPENCL_H
#define SURGSIM_PARALLEL_LINEARSOLVEANDINVERSEOPENCL_H

#include "SurgSim/Math/LinearSparseSolveAndInverse.h"

#include <viennacl/compressed_matrix.hpp>


namespace SurgSim
{
namespace Math
{

class LinearSparseSolveAndInverseOpenCLCG: public LinearSparseSolveAndInverse
{

public:
	LinearSparseSolveAndInverseOpenCLCG();


	virtual void setMatrix(const SparseMatrix& matrix) override;


	virtual Matrix solve(const Matrix& b) const override;


	virtual Matrix getInverse() const override;

private:

	viennacl::compressed_matrix<double> m_vclMatrix;
	Matrix m_eigenMatrix;

};
}
}

#endif