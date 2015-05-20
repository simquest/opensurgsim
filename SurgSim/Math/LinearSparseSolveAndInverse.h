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

#ifndef SURGSIM_MATH_LINEARSPARSESOLVEANDINVERSE_H
#define SURGSIM_MATH_LINEARSPARSESOLVEANDINVERSE_H

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4244)
#endif

#include <Eigen/SparseCore>
#include <unordered_map>

#include <boost/assign/list_of.hpp> // for 'map_list_of()'

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{

/// The linear numerical integration scheme supported
/// Each Linear Solver should have its own entry in this enum
enum LinearSolver
{
	LINEARSOLVER_LU = 0,
	LINEARSOLVER_CONJUGATEGRADIENT
};

const std::unordered_map<LinearSolver, std::string, std::hash<int>> LinearSolverNames =
			boost::assign::map_list_of
			(LINEARSOLVER_LU, "LINEARSOLVER_LU")
			(LINEARSOLVER_CONJUGATEGRADIENT, "LINEARSOLVER_CONJUGATEGRADIENT");

/// LinearSparseSolveAndInverse aims at performing an efficient linear system resolution and
/// calculating its inverse matrix at the same time.
/// This class is very useful in the OdeSolver resolution to improve performance.
/// \sa SurgSim::Math::OdeSolver
class LinearSparseSolveAndInverse
{
public:
	virtual ~LinearSparseSolveAndInverse() {}


	/// Set the linear solver matrix
	/// \param matrix the new matrix to solve/inverse for
	virtual void setMatrix(const SparseMatrix& matrix) = 0;

	/// Solve the linear system (matrix.x=b) using the matrix provided by the latest setMatrix call
	/// for all columns of the rhs matrix b.
	/// \param b The rhs matrix
	/// \return The solution matrix
	virtual Matrix solve(const Matrix& b) const = 0;

	/// \return The linear system's inverse matrix, i.e. the inverse of the matrix provided on the last setMatrix call
	virtual Matrix getInverse() const = 0;
};

/// Derivation for sparse LU solver
class LinearSparseSolveAndInverseLU : public LinearSparseSolveAndInverse
{
public:
	void setMatrix(const SparseMatrix& matrix) override;

	Matrix solve(const Matrix& b) const override;

	Matrix getInverse() const override;

private:
	Eigen::SparseLU<SparseMatrix> m_solver;
};

/// Derivation for sparse CG solver
class LinearSparseSolveAndInverseCG : public LinearSparseSolveAndInverse
{
public:
	/// Set the conjugate gradient convergence tolerance
	/// \param tolerance the new convergence tolerance
	void setTolerance(double tolerance);

	/// Get the conjugate gradient convergence tolerance
	/// \return the convergence tolerance
	double getTolerance();

	/// Set the maximum number of iterations for conjugate gradient
	/// \param iterations the new maximum number of iterations
	void setMaxIterations(SparseMatrix::Index iterations);

	/// Get the conjugate gradient maximum iterations
	/// \return the maximum number of iterations allowed
	SparseMatrix::Index getMaxIterations();

	void setMatrix(const SparseMatrix& matrix) override;

	Matrix solve(const Matrix& b) const override;

	Matrix getInverse() const override;

private:
	Eigen::ConjugateGradient<SparseMatrix> m_solver;
};

}; // namespace Math

}; // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif // SURGSIM_MATH_LINEARSPARSESOLVEANDINVERSE_H
