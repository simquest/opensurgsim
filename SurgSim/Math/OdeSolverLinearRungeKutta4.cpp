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

#include "SurgSim/Math/OdeSolverLinearRungeKutta4.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

OdeSolverLinearRungeKutta4::OdeSolverLinearRungeKutta4(OdeEquation* equation)
	: OdeSolverRungeKutta4(equation),
	  m_initialized(false)
{
	m_name = "Ode Solver Linear Runge Kutta 4";
}

void OdeSolverLinearRungeKutta4::solve(double dt, const OdeState& currentState, OdeState* newState,
									   bool computeCompliance)
{
	if (!m_initialized)
	{
		// The compliance matrix is constant and used in all following calls, so we force its calculation on 1st pass.
		OdeSolverRungeKutta4::solve(dt, currentState, newState, true);
		m_initialized = true;
	}
	else
	{
		// General equation to solve:
		//   M.a(t) = F(t, x(t), v(t)), which is an ode of order 2 that can be reduced to an ode of order 1:
		// y' = (x)' = (          v          ) = f(t, y)
		//      (v)  = (M^-1.F(t, x(t), v(t)))
		// In terms of (x), f(t, (x)) = (v                    )
		//             (v)       (v)    (M^-1.F(t, x(t), v(t)))

		// Runge Kutta 4 computes y(n+1) = y(n) + 1/6.dt.(k1 + 2 * k2 + 2 * k3 + k4)
		// with k1 = f(t(n)       , y(n)            )
		// with k2 = f(t(n) + dt/2, y(n) + k1 * dt/2)
		// with k3 = f(t(n) + dt/2, y(n) + k2 * dt/2)
		// with k4 = f(t(n) + dt  , y(n) + k3 * dt  )

		// 1st evaluate k1 (note that y(n) is currentState)
		m_k1.velocity = currentState.getVelocities();
		// Reminder: m_complianceMatrix = dt.M^-1 (including 0 compliance for all boundary conditions)
		m_equation.update(currentState, OdeEquationUpdate::F);
		m_k1.acceleration = m_equation.applyCompliance(currentState, m_equation.getF() / dt);

		// 2nd evaluate k2
		newState->getPositions()  = currentState.getPositions()  + m_k1.velocity * dt / 2.0;
		newState->getVelocities() = currentState.getVelocities() + m_k1.acceleration * dt / 2.0;
		m_k2.velocity = newState->getVelocities();
		m_equation.update(*newState, OdeEquationUpdate::F);
		m_k2.acceleration = m_equation.applyCompliance(*newState, m_equation.getF() / dt);

		// 3rd evaluate k3
		newState->getPositions()  = currentState.getPositions()  + m_k2.velocity * dt / 2.0;
		newState->getVelocities() = currentState.getVelocities() + m_k2.acceleration * dt / 2.0;
		m_k3.velocity = newState->getVelocities();
		m_equation.update(*newState, OdeEquationUpdate::F);
		m_k3.acceleration = m_equation.applyCompliance(*newState, m_equation.getF() / dt);

		// 4th evaluate k4
		newState->getPositions()  = currentState.getPositions()  + m_k3.velocity * dt;
		newState->getVelocities() = currentState.getVelocities() + m_k3.acceleration * dt;
		m_k4.velocity = newState->getVelocities();
		m_equation.update(*newState, OdeEquationUpdate::F);
		m_k4.acceleration = m_equation.applyCompliance(*newState, m_equation.getF() / dt);

		// Compute the new state using Runge Kutta 4 integration scheme:
		newState->getPositions()  = currentState.getPositions() +
									(m_k1.velocity + m_k4.velocity + 2.0 * (m_k2.velocity + m_k3.velocity)) * dt / 6.0;
		newState->getVelocities() = currentState.getVelocities() +
									(m_k1.acceleration + m_k4.acceleration + 2.0 *
									 (m_k2.acceleration + m_k3.acceleration)) * dt / 6.0;
	}
}

}; // namespace Math

}; // namespace SurgSim
