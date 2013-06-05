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

#ifndef SURGSIM_MATH_MLCPPROBLEM_H
#define SURGSIM_MATH_MLCPPROBLEM_H

#include <vector>
#include <Eigen/Core>
#include <SurgSim/Math/MlcpConstraintType.h>


namespace SurgSim
{
namespace Math
{

/// A description of an MLCP (mixed linear complementarity problem, or mixed LCP) system to be solved.
///
/// A traditional (not mixed!) LCP problem is expressed as \f$\mathbf{A}x + b = c\f$, where \f$x\f$ and \f$c\f$
/// are subject
/// to inequality conditions \f$x_i \ge 0\f$, \f$c_i \ge 0\f$, and \f$x \perp c\f$ (i.e. \f$x \cdot c = 0\f$).
/// Thus for each row \f$i\f$, either
/// * \f$(\mathbf{A}x+b)_i = \mathbf{A}_{i,*}\cdot x+b_i \ge 0\f$ <b>and</b> \f$x_i = 0\f$
///   (the constraint is non-binding, and therefore there is no force); or
/// * \f$(\mathbf{A}x+b)_i = \mathbf{A}_{i,*}\cdot x+b_i = 0\f$ <b>and</b> \f$x_i \ge 0\f$
///   (the constraint is binding, and therefore there is a positive force to enforce the constraint)
/// Solving the problem produces the vector \f$x\f$, from which \f$c\f$ can also be computed if needed.
///
/// A mixed LCP problem is defined in the same way, except that for a certain subset of indices, the conditions are
/// restricted further to require \f$c_i\f$ to be zero, \f$\mathbf{A}_{i,*}\cdot x+b_i = 0\f$,
/// which results in a non-zero force,
/// \f$x_i \ge 0\f$.  These are referred to as <i>bilateral</i> constraints, as opposed to the <i>unilateral</i>
/// constraints that behave as they do in the LCP problem.
///
// TODO(advornik): Describe why friction is special!!!
// TODO(advornik): Get rid of the constraint types and encode necessary info in other ways.
struct MlcpProblem
{
	/// Matrix \f$\mathbf{A}\f$ used to describe the mixed LCP problem.
	Eigen::MatrixXd A;
	/// Vector \f$b\f$ used to describe the mixed LCP problem.
	Eigen::VectorXd b;
	/// A vector of friction coefficients used to describe the mixed LCP problem.
	/// \todo This API will change in the future to something more independent of physics.
	Eigen::VectorXd mu;
	/// A vector of constraint types used to describe the mixed LCP problem.
	/// \todo This API will change in the future to something more independent of physics.
	std::vector<MlcpConstraintType> constraintTypes;

	MlcpProblem()
	{
	}

	MlcpProblem(const MlcpProblem& other) :
		A(other.A),
		b(other.b),
		mu(other.mu),
		constraintTypes(other.constraintTypes)
	{
	}

	MlcpProblem(MlcpProblem&& other) :
		A(std::move(other.A)),
		b(std::move(other.b)),
		mu(std::move(other.mu)),
		constraintTypes(std::move(other.constraintTypes))
	{
	}

	MlcpProblem& operator= (const MlcpProblem& other)
	{
		A = other.A;
		b = other.b;
		mu = other.mu;
		constraintTypes = other.constraintTypes;
		return *this;
	}

	MlcpProblem& operator= (MlcpProblem&& other)
	{
		A = std::move(other.A);
		b = std::move(other.b);
		mu = std::move(other.mu);
		constraintTypes = std::move(other.constraintTypes);
		return *this;
	}

	int getSize() const
	{
		return b.rows();
	}

	bool isConsistent() const
	{
		int numConstraintTypes = constraintTypes.size();
		return ((b.rows() >= 0) && (b.cols() == 1) && (A.rows() == b.rows()) && (A.cols() == A.rows()) &&
		        (numConstraintTypes <= b.rows()) && (mu.size() == numConstraintTypes));
	}
};

};  // namespace Math
};  // namespace SurgSim

#endif // SURGSIM_MATH_MLCPPROBLEM_H
