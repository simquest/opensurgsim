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

#include <Eigen/core>
#include <Eigen/Sparse>

#include <SurgSim/Math/OdeEquation.h>
#include <SurgSim/Math/LinearSolveAndInverse.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

namespace SurgSim
{

namespace Math
{

/// Base class for all solvers of ode equation of order 2 of the form M(x(t), v(t)).a(t) = f(t, x(t), v(t))
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.f(x, v) )
/// \note To allow the use of explicit and implicit solver, we need to be able to evaluate
/// \note f(t, x(t), v(t)) but also
/// \note K = -df/dx(t, x(t), v(t))
/// \note D = -df/dv(t, x(t), v(t))
/// \note Models wanting the use of implicit solvers will need to compute these Jacobians matrices.
/// \note Also, the matrices types are all templatized to allow for optimization
/// \tparam State Type of the state y=(x v)
/// \tparam MType Type of the M matrix
/// \tparam DType Type of the D matrix
/// \tparam KType Type of the K matrix
/// \tparam SType Type of the system matrix
/// \note State type is expected to have the methods
/// \note Vector& getPositions();
/// \note Vector& getVelocities();
template <class State, class MType, class DType, class KType, class SType>
class OdeSolver
{
public:
	typedef Eigen::Matrix<double, Eigen::Dynamic,              1, Eigen::DontAlign> Vector;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Matrix;
	typedef SurgSim::Math::OdeEquation<State, MType, DType, KType, SType> OdeEquation;

	/// Constructor
	/// \param equation The ode equation to be solved
	/// \param initialState The initial state
	OdeSolver(const OdeEquation& equation, const State& initialState);
	
	/// Gets the solver's name
	/// \return The solver name
	virtual const std::string getName() const = 0;

	/// Solves the equation
	/// \param dt The time step
	/// \param currentState State at time t
	/// \param[out] newState State at time t+dt
	virtual void solve(double dt, const State& currentState, State* newState) = 0;
	
	/// Queries the current M matrix
	/// \return The latest M matrix calculated
	const MType& getM() const;

	/// Queries the current D matrix
	/// \return The latest D matrix calculated
	const DType& getD() const;

	/// Queries the current K matrix
	/// \return The latest K matrix calculated
	const KType& getK() const;

	/// Queries the current system matrix
	/// \return The latest system matrix calculated
	const SType& getSystemMatrix() const;

	/// Queries the current compliance matrix
	/// \return The latest compliance matrix calculated
	const Matrix& getCompliance() const;

	/// Queries the current value of f
	/// \return The latest vector f calculated
	const Vector& getF() const;

	/// Queries the current value of a
	/// \return The latest vector a calculated
	const Vector& getA() const;

protected:

	/// Allocates the data structure
	/// \param size The size to account for in the data structure
	void allocate(unsigned int size);

	/// Computes vector m_f (to keep the API computeM/D/K/F consistent)
	/// \param currentState The state to compute the vector with
	void computeF(const State& currentState);

	/// Computes matrix m_M, optimizing computation for constant/non-constant cases
	/// \param currentState The state to compute the matrix with (if needed)
	void computeM(const State& currentState);

	/// Computes matrix m_D, optimizing computation for constant/non-constant cases
	/// \param currentState The state to compute the matrix with (if needed)
	void computeD(const State& currentState);

	/// Computes matrix m_K, optimizing computation for constant/non-constant cases
	/// \param currentState The state to compute the matrix with (if needed)
	void computeK(const State& currentState);

	/// The ode equation (API providing the necessary evaluation methods)
	const OdeEquation& m_equation;
	
	/// The initial state (useful for static resolution)
	const State& m_initialState;

	/// Vector f, contains latest evaluation of f(x,v)
	Vector m_f;
	
	/// Vector a, contains latest evaluation of M^{-1}.f(x,v)
	Vector m_a;

	/// Matrix M
	MType m_M;

	/// Matrix D = -df/dv
	DType m_D;

	/// Matrix K = -df/dx
	KType m_K;
	
	/// System matrix (can be M, K, combination of MDK depending on the solver)
	SType m_systemMatrix;

	/// The compliance matrix is the inverse of the system matrix
	/// Compliance is always a dense matrix (full matrix unless we have a diagonal matrix)
	Matrix m_compliance;

	/// If M is a constant matrix, this variable keeps track of its initialization
	bool m_constantMinitialized;

	/// If D is a constant matrix, this variable keeps track of its initialization
	bool m_constantDinitialized;

	/// If K is a constant matrix, this variable keeps track of its initialization
	bool m_constantKinitialized;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVER_H