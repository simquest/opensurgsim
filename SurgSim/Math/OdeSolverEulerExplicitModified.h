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

#ifndef SURGSIM_MATH_ODESOLVEREULEREXPLICITMODIFIED_H
#define SURGSIM_MATH_ODESOLVEREULEREXPLICITMODIFIED_H

#include <SurgSim/Math/OdeSolver.h>
#include <SurgSim/Math/LinearSolveAndInverse.h>

namespace SurgSim
{

namespace Math
{

/// Euler Explicit Modified ode solver
/// \note M(x(t), v(t)).a(t) = f(t, x(t), v(t))
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.f(x, v) )
/// \note By simply using the newly computed velocity in the position update, the method gains in stability:
/// \note { x(t+dt) = x(t) + dt.v(t+dt)
/// \note { v(t+dt) = v(t) + dt.a(t)
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
class ModifiedExplicitEuler : public OdeSolver<State, MT, DT, KT, ST>
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	/// \param initialState The initial state
	ModifiedExplicitEuler(OdeEquation<State, MT, DT, KT, ST>& equation, const State& initialState);

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	void solve(double dt, const State& currentState, State* newState) override;

private:
	/// Helper class to solve and inverse a system of linear equations
	/// Optimized with the matrix type
	SolveAndInverse<MT> m_solveAndInverse;

	/// Temporary matrix to modify the mass matrix into the system matrix while keeping the proper type
	MT m_MsystemMatrix;
};

}; // namespace Math

}; // namespace SurgSim

#include <SurgSim/Math/OdeSolverEulerExplicitModified-inl.h>

#endif // SURGSIM_MATH_ODESOLVEREULEREXPLICITMODIFIED_H
