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
#include "viennacl/linalg/cg.hpp"
#include <viennacl/linalg/matrix_operations.hpp>
#include "boost/thread/lock_guard.hpp"


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
	static boost::mutex m;
	m_eigenMatrix = matrix;
	m_inverse = m_eigenMatrix.inverse();

	//boost::lock_guard<boost::mutex> guard(m);
	viennacl::copy(m_inverse, m_vclMatrix);
}

Matrix LinearSparseSolveAndInverseOpenCLCG::solve(const Matrix& b) const
{
	// odd declaring these as class members causes the compilation to fail
	viennacl::vector<double> vcl_rhs;

	Vector v = static_cast<Vector>(b);

	viennacl::copy(v, vcl_rhs);
	//vcl_solution = viennacl::linalg::solve(m_vclMatrix, vcl_rhs, viennacl::linalg::cg_tag());

	vcl_rhs = viennacl::linalg::prod(m_vclMatrix, vcl_rhs);

	Vector result(b.size());
	viennacl::copy(vcl_rhs, result);
	return result;
}

Matrix LinearSparseSolveAndInverseOpenCLCG::getInverse() const
{
	return m_eigenMatrix.inverse();
}

}
}