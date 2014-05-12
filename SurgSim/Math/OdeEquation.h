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

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{

class OdeState;

/// Ode equation of 2nd order of the form M(x,v).a = F(x, v) with (x0, v0) for initial conditions
/// and a set of boundary conditions. The problem is called a Boundary Value Problem (BVP).
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.F(x, v) )
/// \note To allow the use of explicit and implicit solver, we need to be able to evaluate
/// \note M(x,v), F(x,v) but also K = -dF/dx(x,v), D = -dF/dv(x,v)
/// \note Models wanting the use of implicit solvers will need to compute these Jacobian matrices.
class OdeEquation
{
public:
	/// Virtual destructor
	virtual ~OdeEquation(){}

	/// Retrieves the ode initial conditions (x0, v0) (i.e. the initial state)
	/// \return The initial state
	const std::shared_ptr<OdeState> getInitialState() const;

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	virtual Vector& computeF(const OdeState& state) = 0;

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	virtual const Matrix& computeM(const OdeState& state) = 0;

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	virtual const Matrix& computeD(const OdeState& state) = 0;

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	virtual const Matrix& computeK(const OdeState& state) = 0;

	/// Evaluation of f(x,v), M(x,v), D = -df/dv(x,v), K = -df/dx(x,v)
	/// When all the terms are needed, this method can perform optimization in evaluating everything together
	/// \param state (x, v) the current position and velocity to evaluate the various terms with
	/// \param[out] f The RHS f(x,v)
	/// \param[out] M The matrix M(x,v)
	/// \param[out] D The matrix D = -df/dv(x,v)
	/// \param[out] K The matrix K = -df/dx(x,v)
	/// \note Returns pointers, the internal data will remain unchanged until the next call to computeFMDK() or
	/// \note computeF(), computeM(), computeD(), computeK()
	virtual void computeFMDK(const OdeState& state, Vector** f, Matrix** M, Matrix** D, Matrix** K) = 0;

protected:
	/// The initial state (which defines the ODE initial conditions (x0, v0))
	/// \note MUST be set by the derived classes
	std::shared_ptr<OdeState> m_initialState;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODEEQUATION_H
