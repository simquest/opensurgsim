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

#ifndef SURGSIM_MATH_LINEARSOLVEANDINVERSE_H
#define SURGSIM_MATH_LINEARSOLVEANDINVERSE_H

#include <Eigen/SparseCore>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{

namespace Math
{

/// LinearSparseSolveAndInverse aims at performing an efficient linear system resolution and
/// calculating its inverse matrix at the same time.
/// This class is very useful in the OdeSolver resolution to improve performance.
/// \sa SurgSim::Math::OdeSolver
class LinearSparseSolveAndInverse
{
public:
	virtual ~LinearSparseSolveAndInverse() {}

	/// Solve a linear system A.x=b and compute the matrix A^-1
	/// \param A Linear system matrix
	/// \param b Linear system right-hand-side
	/// \param[out] x Linear system unknown (if requested)
	/// \param[out] Ainv Linear system matrix inverse = A^-1 (if requested)
	virtual void operator()(const SparseMatrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) = 0;
};

/// Derivation for sparse LU solver
class LinearSparseSolveAndInverseLU : public LinearSparseSolveAndInverse
{
public:
	void operator()(const SparseMatrix& A, const Vector& b, Vector* x = nullptr, Matrix* Ainv = nullptr) override;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_LINEARSOLVEANDINVERSE_H
