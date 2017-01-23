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

#include "Modules/Parallel/LinearSparseSolveAndInverseOpenCl.h"

#include <boost/compute/system.hpp>
#include "viennacl/vector.hpp"
#include "viennacl/linalg/cg.hpp"

namespace SurgSim
{
namespace Math
{

LinearSparseSolveAndInverseOpenCLCG::LinearSparseSolveAndInverseOpenCLCG()
{
	SURGSIM_ASSERT(boost::compute::system::devices().size() > 0)
			<< "Need at least on device for OpenCL solution";
}

void LinearSparseSolveAndInverseOpenCLCG::setMatrix(const SparseMatrix& matrix)
{
	m_eigenMatrix = matrix;
	viennacl::copy(matrix, m_vclMatrix);
}

Matrix LinearSparseSolveAndInverseOpenCLCG::solve(const Matrix& b) const
{
	// Try what happens when preallocating these
	viennacl::vector<double> vcl_rhs(b.size());
	viennacl::vector<double> vcl_solution(b.size());

	Vector v = static_cast<Vector>(b);

	viennacl::copy(v, vcl_rhs);

	vcl_solution = viennacl::linalg::solve(m_vclMatrix, vcl_rhs, viennacl::linalg::cg_tag());

	Vector result(b.size());
	viennacl::copy(vcl_solution, result);
	return result;
}

Matrix LinearSparseSolveAndInverseOpenCLCG::getInverse() const
{
	return m_eigenMatrix.inverse();
}

}
}