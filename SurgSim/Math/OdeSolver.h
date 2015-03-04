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

#include <functional>
#include <unordered_map>

#include <boost/assign/list_of.hpp> // for 'map_list_of()'

#include "SurgSim/Math/LinearSolveAndInverse.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeEquation.h"

namespace SurgSim
{

namespace Math
{

/// The diverse numerical integration scheme supported
/// Each Ode Solver should have its own entry in this enum
enum IntegrationScheme
{
	INTEGRATIONSCHEME_STATIC = 0,
	INTEGRATIONSCHEME_LINEAR_STATIC,
	INTEGRATIONSCHEME_EXPLICIT_EULER,
	INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER,
	INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
	INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER,
	INTEGRATIONSCHEME_IMPLICIT_EULER,
	INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER,
	INTEGRATIONSCHEME_RUNGE_KUTTA_4,
	INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4
};

const std::unordered_map<IntegrationScheme, std::string, std::hash<int>> IntegrationSchemeNames =
			boost::assign::map_list_of
			(INTEGRATIONSCHEME_STATIC, "INTEGRATIONSCHEME_STATIC")
			(INTEGRATIONSCHEME_LINEAR_STATIC, "INTEGRATIONSCHEME_LINEAR_STATIC")
			(INTEGRATIONSCHEME_EXPLICIT_EULER, "INTEGRATIONSCHEME_EXPLICIT_EULER")
			(INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER, "INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER")
			(INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER, "INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER")
			(INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER, "INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER")
			(INTEGRATIONSCHEME_IMPLICIT_EULER, "INTEGRATIONSCHEME_IMPLICIT_EULER")
			(INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER, "INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER")
			(INTEGRATIONSCHEME_RUNGE_KUTTA_4, "INTEGRATIONSCHEME_RUNGE_KUTTA_4")
			(INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4, "INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4");

/// Base class for all solvers of ode equation of order 2 of the form \f$M(x(t), v(t)).a(t) = f(t, x(t), v(t))\f$. <br>
/// This ode equation is solved as an ode of order 1 by defining the state vector
/// \f$y = \left(\begin{array}{c}x\\v\end{array}\right)\f$:
/// \f[
///   y' = \left(\begin{array}{c} x' \\ v' \end{array}\right) =
///   \left(\begin{array}{c} v \\ M(x, v)^{-1}.f(t, x, v) \end{array}\right)
/// \f]
/// \note To allow the use of explicit and implicit solver, we need to be able to evaluate:
/// \note \f$M(x(t), v(t))\f$
/// \note \f$f(t, x(t), v(t))\f$ but also
/// \note \f$K = -\frac{\partial f}{\partial x}(x(t), v(t))\f$
/// \note \f$D = -\frac{\partial f}{\partial v}(x(t), v(t))\f$
/// \note Models wanting the use of implicit solvers will need to compute these Jacobian matrices.
/// \note Matrices all have dense storage, but a specialized linear solver can be set per solver.
class OdeSolver
{
public:
	/// Constructor
	/// \param equation The ode equation to be solved
	explicit OdeSolver(OdeEquation* equation);

	/// Virtual destructor
	virtual ~OdeSolver()
	{}

	/// Gets the solver's name
	/// \return The solver name
	const std::string getName() const;

	/// Sets the specialized linear solver to use with this Ode solver
	/// \param linearSolver the linear solver to use when solving the ode equation
	void setLinearSolver(std::shared_ptr<LinearSolveAndInverse> linearSolver);

	/// Gets the specialized linear solver used with this Ode solver
	/// \return The linear solver used when solving the ode equation
	std::shared_ptr<LinearSolveAndInverse> getLinearSolver() const;

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	virtual void solve(double dt, const OdeState& currentState, OdeState* newState) = 0;

	/// Queries the current system matrix
	/// \return The latest system matrix calculated
	const SparseMatrix& getSystemMatrix() const;

	/// Queries the current compliance matrix
	/// \return The latest compliance matrix calculated
	const Matrix& getCompliance() const;

protected:
	/// Allocates the system and compliance matrices
	/// \param size The size to account for in the data structure
	void allocate(size_t size);

	/// Name for this solver
	/// \note MUST be set by the derived classes
	std::string m_name;

	/// The ode equation (API providing the necessary evaluation methods and the initial state)
	OdeEquation& m_equation;

	/// The specialized linear solver to use when solving the ode equation
	std::shared_ptr<LinearSolveAndInverse> m_linearSolver;

	/// System matrix (can be M, K, combination of MDK depending on the solver)
	/// \note A static solver will have K for system matrix
	/// \note A dynamic explicit solver will have M for system matrix
	/// \note A dynamic implicit solver will have a combination of M, D and K for system matrix
	SparseMatrix m_systemMatrix;

	/// Compliance matrix which is the inverse of the system matrix
	Matrix m_compliance;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVER_H
