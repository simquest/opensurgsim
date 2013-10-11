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

#ifndef SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H
#define SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H

#include <SurgSim/Math/OdeSolver.h>
#include <SurgSim/Math/LinearSolveAndInverse.h>

namespace SurgSim
{

namespace Math
{

/// Euler Implicit ode solver
/// { x(t+dt) = x(t) + dt.v(t+dt)
/// { v(t+dt) = v(t) + dt.a(t+dt)
template <class State, class MType, class DType, class KType, class SType>
class ImplicitEuler : public OdeSolver<State, MType, DType, KType, SType>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	/// \param initialState The initial state
	ImplicitEuler(const OdeEquation& equation, const State& initialState) :
		OdeSolver(equation, initialState)
	{
	}

	/// Gets the solver's name
	/// \return The solver name
	const std::string getName() const override
	{
		return "Implicit Euler";
	}

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	void solve(double dt, const State& currentState, State* newState) override
	{
		// General equation to solve:
		//   M.a(t+dt) = f(t+dt, x(t+dt), v(t+dt))
		//   M.a(t+dt) = f(t) + df/dx.deltaX + df/dv.deltaV
		// Note that K = -df/dx and D = -df/dv
		// Compliance matrix on the velocity level:
		//   (M.deltaV)/dt = f(t) - K.(dt.v(t) + dt.deltaV) - D.deltaV
		//   (M/dt + D + dt.K).deltaV = f(t) - dt.K.v(t)
		// Resolution for the acceleration:
		//   (M + dt.D + dt^2.K).a = f(t) - dt.K.v(t)

		// Computes m_M = M
		computeM(currentState);
		// Computes m_D = D
		computeD(currentState);
		// Computes m_K = K
		computeK(currentState);
		// Computes m_f = f(t, x(t), v(t))
		computeF(currentState);

		// Adds the Euler Implicit terms on the right-hand-side
		m_f -= (m_K * currentState.getVelocities()) * dt;

		// Computes the system matrix (left-hand-side matrix)
		m_systemMatrix  = m_M * (1.0 / dt);
		m_systemMatrix += m_D;
		m_systemMatrix += m_K * dt;

		// Computes deltaV (stored in m_a) and m_compliance = 1/m_systemMatrix
		Vector& deltaV = m_a;
		m_solveAndInverse.solveAndComputeInverse(m_systemMatrix, m_f, &deltaV, &m_compliance);

		// Compute the new state using the Euler Implicit scheme:
		newState->getVelocities() = currentState.getVelocities() + deltaV;
		newState->getPositions()  = currentState.getPositions()  + dt * newState->getVelocities();

		// Adjust the variable m_a to contain accelerations: m_a = deltaV / dt
		m_a /= dt;
	}

private:
	/// Helper class to solve and inverse a system of linear equations
	/// Optimized with the matrix type
	SolveAndInverse<SType> m_solveAndInverse;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVEREULERIMPLICIT_H