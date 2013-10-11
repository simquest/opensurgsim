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

#include <Eigen/core>
#include <Eigen/Sparse>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

namespace SurgSim
{

namespace Math
{

/// Ode equation of order 2 of the form M(x,v).a = F(x, v)
/// \note This ode equation is solved as an ode of order 1 by defining the state vector y = (x v)^t:
/// \note y' = ( x' ) = ( dx/dt ) = (       v        )                   
/// \note      ( v' ) = ( dv/dt ) = ( M(x, v)^{-1}.F(x, v) )
/// \note To allow the use of explicit and implicit solver, we need to be able to evaluate
/// \note F(x,v) but also dF/dx(x,v) = -K, dF/dv(x,v) = -D
/// \note Models wanting the use of implicit solvers will need to compute these Jacobians matrices.
/// \note Also, the matrices types are all templatized to allow for optimization
/// \tparam State Type of the state y=(x v)
/// \tparam MType Type of the M matrix
/// \tparam DType Type of the D matrix
/// \tparam KType Type of the K matrix
/// \tparam SType Type of the system matrix
template <class State, class MType, class DType, class KType, class SType>
class OdeEquation
{
public:
	typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign> Vector;

	/// RHS vector f for a given state
	/// \param state (x, v) the current position and velocity to evaluate the vector f with
	/// \param[out] f The vector f to store the result into
	virtual void computeF(const State& state, Vector* F) = 0;

	/// LHS matrix M for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M with
	/// \param[out] M The matrix M to store the result into
	virtual void computeM(const State& state, MType* M) = 0;
	/// Is the matrix M independent of the state or not ?
	/// \return True if M is a constant matrix over time, False otherwise
	virtual bool isMConstant() const = 0;
		
	/// Opposite of the 1st derivative of f w.r.t. v for a given state: D = -df/dv (x,v)
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \param[out] The matrix D = -df/dv to store the result into
	virtual void computeD(const State& state, DType* D) = 0;
	/// Is the matrix D independent of the state or not ?
	/// \return True if D is a constant matrix over time, False otherwise
	virtual bool isDConstant() const = 0;

	/// Opposite of the 1st derivative of f w.r.t. x for a given state: K = - df/dx (x,v)
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \param[out] The matrix K = -df/dx to store the result into
	virtual void computeK(const State& state, KType* dFdx) = 0;
	/// Is the matrix K independent of the state or not ?
	/// \return True if K is a constant matrix over time, False otherwise
	virtual bool isKConstant() const = 0;

	/// Helper methods to allocate the various matrices type with the proper size
	/// \param[in,out] M, D, K, systemMatrix matrices to resize
	/// \param size The size to account for
	virtual void resizeMatricesToFitState(MType& M, DType& D, KType& K, SType& systemMatrix, unsigned int size) = 0;
	/// Helper methods to allocate a vector with the proper size
	/// \param[in,out] x vector to resize
	/// \param size The size to account for
	virtual void resizeVectorToFitState(Vector& x, unsigned int size) = 0;
};

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODEEQUATION_H