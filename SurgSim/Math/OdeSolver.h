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

#ifndef SURGSIM_MATH_ODESOLVER_H
#define SURGSIM_MATH_ODESOLVER_H

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

#include <SurgSim/Math/OdeEquation.h>

namespace SurgSim
{

namespace Math
{

/// Base class for all solvers of ode equation of order 2 of the form M(x(t), v(t)).a(t) = f(t, x(t), v(t))
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.f(x, v) )
/// \note To allow the use of explicit and implicit solver, we need to be able to evaluate
/// \note M(x(t), v(t))
/// \note f(t, x(t), v(t)) but also
/// \note K = -df/dx(x(t), v(t))
/// \note D = -df/dv(x(t), v(t))
/// \note Models wanting the use of implicit solvers will need to compute these Jacobian matrices.
/// \note Also, the matrices types are all templatized to allow for optimization
/// \tparam State Type of the state y=(x v)
/// \tparam MT Type of the matrix M
/// \tparam DT Type of the matrix D
/// \tparam KT Type of the matrix K
/// \tparam ST Type of the system matrix (linear combination of M, D, K)
/// \note State is expected to hold on to the dof derivatives and have the API:
/// \note   Vector& getPositions();
/// \note   Vector& getVelocities();
/// \note   Vector& getAccelerations();
template <class State, class MT, class DT, class KT, class ST>
class OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	/// \param initialState The initial state (can be useful for static resolution for example)
	OdeSolver(OdeEquation<State, MT, DT, KT, ST>& equation, const State& initialState);

	/// Virtual destructor
	virtual ~OdeSolver()
	{}

	/// Gets the solver's name
	/// \return The solver name
	const std::string getName() const;

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	virtual void solve(double dt, const State& currentState, State* newState) = 0;

	/// Queries the current system matrix
	/// \return The latest system matrix calculated
	const ST& getSystemMatrix() const;

	/// Queries the current compliance matrix
	/// \return The latest compliance matrix calculated
	const Matrix& getCompliance() const;

protected:
	/// Name for this solver (will be given by the derived classes)
	std::string m_name;

	/// Allocates the system and compliance matrices
	/// \param size The size to account for in the data structure
	void allocate(unsigned int size);

	/// The ode equation (API providing the necessary evaluation methods)
	OdeEquation<State, MT, DT, KT, ST>& m_equation;

	/// The initial state (useful for static resolution)
	const State& m_initialState;

	/// System matrix (can be M, K, combination of MDK depending on the solver)
	ST m_systemMatrix;

	/// Compliance matrix which is the inverse of the system matrix
	/// Compliance is always a dense matrix (full matrix unless we have a diagonal matrix)
	Matrix m_compliance;
};

}; // namespace Math

}; // namespace SurgSim

#include <SurgSim/Math/OdeSolver-inl.h>

#endif // SURGSIM_MATH_ODESOLVER_H
