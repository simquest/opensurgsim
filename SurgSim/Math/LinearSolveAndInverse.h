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

#ifndef SURGSIM_MATH_SOLVEANDINVERSE_H
#define SURGSIM_MATH_SOLVEANDINVERSE_H

namespace SurgSim
{

namespace Math
{

template <class M>
class SolveAndInverse
{
public:
	typedef Eigen::Matrix<double,Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

	SolveAndInverse(){}

	void solveAndComputeInverse(const M& m, const Vector& b, Vector* x, Matrix* inv)
	{
		SURGSIM_ASSERT(false) << "Unknown matrix type for SolveAndInverse";
	}
};

template <>
class SolveAndInverse<Eigen::DiagonalMatrix<double,Eigen::Dynamic>>
{
private:
	Eigen::DiagonalMatrix<double,Eigen::Dynamic> m_Minv;

public:
	typedef Eigen::Matrix<double,Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

	SolveAndInverse(){}

	void solveAndComputeInverse(const Eigen::DiagonalMatrix<double,Eigen::Dynamic>& M, const Vector& b, Vector* x, Matrix* inv)
	{
		(*inv) = m_Minv = M.inverse();
		(*x) = m_Minv * b;
	}
};

template <>
class SolveAndInverse<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>>
{
private:
public:
	typedef Eigen::Matrix<double,Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

	SolveAndInverse(){}

	void solveAndComputeInverse(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& M, const Vector& b, Vector* x, Matrix* inv)
	{
		Eigen::PartialPivLU<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> lu(M);
		(*x) = lu.solve(b);
		(*inv) = lu.inverse();
	}
};

template <>
class SolveAndInverse<Eigen::SparseMatrix<double,Eigen::ColMajor>>
{
private:
	Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> identity;

public:
	typedef Eigen::Matrix<double,Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;

	SolveAndInverse(){}

	void solveAndComputeInverse(const Eigen::SparseMatrix<double,Eigen::ColMajor>& M, const Vector& b, Vector* x, Matrix* inv)
	{
		Eigen::SparseLU<Eigen::SparseMatrix<double, Eigen::ColMajor>> solver;
		// Compute the ordering permutation vector from the structural pattern of A
		solver.analyzePattern(M);
		// Compute the numerical factorization
		solver.factorize(M);
		//Use the factors to solve the linear system
		(*x) = solver.solve(b); 

		if (identity.rows() != M.rows())
		{
			identity.resize(M.rows(), M.cols());
			identity.setIdentity();
		}
		(*inv) = solver.solve(identity);
	}
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_SOLVEANDINVERSE_H