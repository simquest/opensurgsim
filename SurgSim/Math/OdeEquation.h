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

#ifndef SURGSIM_MATH_ODEEQUATION_H
#define SURGSIM_MATH_ODEEQUATION_H

#include <memory>

#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{

class OdeState;

/// Ode equation of 2nd order of the form \f$M(x,v).a = F(x, v)\f$ with \f$(x0, v0)\f$ for initial conditions
/// and a set of boundary conditions. The problem is called a Boundary Value Problem (BVP).
/// This ode equation is solved as an ode of order 1 by defining the state vector
/// \f$y = \left(\begin{array}{c}x\\v\end{array}\right)\f$:
/// \f[
///   y' = \left(\begin{array}{c} x' \\ v' \end{array}\right) =
///   \left(\begin{array}{c} v \\ M(x, v)^{-1}.f(t, x, v) \end{array}\right)
/// \f]
/// \note To allow the use of explicit and implicit solver, we need to be able to evaluate
/// \note \f$M(x, v)\f$, \f$f(t, x, v)\f$ but also \f$K = -dF/dx(x, v)\f$ and \f$D = -dF/dv(x, v)\f$
/// \note Models wanting the use of implicit solvers will need to compute these Jacobian matrices.
class OdeEquation
{
public:
	/// Virtual destructor
	virtual ~OdeEquation() {}

	/// Retrieves the ode initial conditions \f$(x0, v0)\f$ (i.e the initial state)
	/// \return The initial state
	const std::shared_ptr<OdeState> getInitialState() const;

	/// Evaluation of the RHS function \f$f(x, v)\f$ for a given state
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the function \f$f(x,v)\f$ with
	/// \return The vector containing \f$f(x, v)\f$
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	virtual Vector& computeF(const OdeState& state) = 0;

	/// Evaluation of the LHS matrix \f$M(x,v)\f$ for a given state
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the matrix \f$M(x,v)\f$ with
	/// \return The matrix \f$M(x,v)\f$
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	virtual const SparseMatrix& computeM(const OdeState& state) = 0;

	/// Evaluation of \f$D = -\frac{\partial f}{\partial v}(x,v)\f$ for a given state
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix \f$D = -\frac{\partial f}{\partial v}(x,v)\f$
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	virtual const SparseMatrix& computeD(const OdeState& state) = 0;

	/// Evaluation of \f$K = -\frac{\partial f}{\partial x}(x,v)\f$ for a given state
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix \f$K = -\frac{\partial f}{\partial x}(x,v)\f$
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	virtual const SparseMatrix& computeK(const OdeState& state) = 0;

	/// Evaluation of \f$f(x,v)\f$, \f$M(x,v)\f$, \f$D = -\frac{\partial f}{\partial v}(x,v)\f$ and
	/// \f$K = -\frac{\partial f}{\partial x}(x,v)\f$.
	/// When all the terms are needed, this method can perform optimization in evaluating everything together
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the various terms with
	/// \param[out] f The RHS \f$f(x,v)\f$
	/// \param[out] M The matrix \f$M(x,v)\f$
	/// \param[out] D The matrix \f$D = -\frac{\partial f}{\partial v}(x,v)\f$
	/// \param[out] K The matrix \f$K = -\frac{\partial f}{\partial x}(x,v)\f$
	/// \note Returns pointers, the internal data will remain unchanged until the next call to computeFMDK() or
	/// \note computeF(), computeM(), computeD(), computeK()
	virtual void computeFMDK(const OdeState& state, Vector** f, SparseMatrix** M, SparseMatrix** D,
							 SparseMatrix** K) = 0;

	/// Calculate the product C.b where C is the compliance matrix with boundary conditions
	/// applied. Note that this can be rewritten as (Bt)(M^-1)(B.b) = (Bt)((M^-1)(B.b)) = x,
	/// where (M^-1)(B.b) = y is simply the solution to M.y = B.b and Bt.y = x.
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the various terms with
	/// \param b The input matrix \b
	/// \return C.b
	virtual Matrix applyCompliance(const OdeState& state, const Matrix& b) = 0;

protected:
	/// The initial state (which defines the ODE initial conditions \f$(x0, v0)\f$)
	/// \note MUST be set by the derived classes
	std::shared_ptr<OdeState> m_initialState;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODEEQUATION_H
