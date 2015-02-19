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

#ifndef SURGSIM_MATH_UNITTESTS_MOCKOBJECT_H
#define SURGSIM_MATH_UNITTESTS_MOCKOBJECT_H

#include "SurgSim/Math/OdeEquation.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{

namespace Math
{

class MassPointState : public OdeState
{
public:
	MassPointState() : OdeState()
	{
		setNumDof(3, 1);
		getPositions().setLinSpaced(1.0, 1.3);
		getVelocities().setLinSpaced(0.4, -0.3);
	}
};

class MassPoint : public OdeEquation
{
public:
	/// Constructor
	/// \param viscosity The mass viscosity
	explicit MassPoint(double viscosity = 0.0) :
		m_mass(0.456),
		m_viscosity(viscosity),
		m_gravity(0.0, -9.81, 0.0),
		m_f(3),
		m_M(3, 3),
		m_D(3, 3),
		m_K(3, 3)
	{
		this->m_initialState = std::make_shared<MassPointState>();
	}

	void disableGravity()
	{
		m_gravity.setZero();
	}

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	Vector& computeF(const OdeState& state) override
	{
		m_f = m_mass * m_gravity - m_viscosity * state.getVelocities();
		return m_f;
	}

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	const Matrix& computeM(const OdeState& state) override
	{
		m_M.setIdentity();
		m_M *= m_mass;
		return m_M;
	}

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	const Matrix& computeD(const OdeState& state) override
	{
		m_D.setIdentity();
		m_D *= m_viscosity;
		return m_D;
	}

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	const Matrix& computeK(const OdeState& state)
	{
		m_K.setZero();
		return m_K;
	}

	/// Evaluation of f(x,v), M(x,v), D = -df/dv(x,v), K = -df/dx(x,v)
	/// When all the terms are needed, this method can perform optimization in evaluating everything together
	/// \param state (x, v) the current position and velocity to evaluate the various terms with
	/// \param[out] f The RHS f(x,v)
	/// \param[out] M The matrix M(x,v)
	/// \param[out] D The matrix D = -df/dv(x,v)
	/// \param[out] K The matrix K = -df/dx(x,v)
	/// \note Returns pointers, the internal data will remain unchanged until the next call to computeFMDK() or
	/// \note computeF(), computeM(), computeD(), computeK()
	void computeFMDK(const OdeState& state, Vector** f, Matrix** M, Matrix** D, Matrix** K) override
	{
		m_M.setIdentity();
		m_M *= m_mass;
		m_D.setIdentity();
		m_D *= m_viscosity;
		m_K.setZero();
		m_f = m_mass * m_gravity - m_viscosity * state.getVelocities();

		*f = &m_f;
		*K = &m_K;
		*D = &m_D;
		*M = &m_M;
	}

	double m_mass, m_viscosity;
	Vector3d m_gravity;
	Vector m_f;
	Matrix m_M, m_D, m_K;
};



/// State class for static resolution
/// It contains 3 nodes with 3 dofs each, with positions (0 0 0) (1 0 0) (2 0 0) and null velocities/accelerations
class MassPointsStateForStatic : public OdeState
{
public:
	MassPointsStateForStatic() : OdeState()
	{
		setNumDof(3, 3);
		getPositions().segment<3>(3) = Vector3d(1.0, 0.0, 0.0);
		getPositions().segment<3>(6) = Vector3d(2.0, 0.0, 0.0);
	}
};

// Model of 3 nodes connected by springs with the 1st node fixed (no mass, no damping, only deformations)
class MassPointsForStatic : public SurgSim::Math::OdeEquation
{
public:
	/// Constructor
	MassPointsForStatic() :
		m_f(9),
		m_gravityForce(9),
		m_K(9, 9)
	{
		m_f.setZero();
		m_K.setZero();
		m_gravityForce.setZero();
		m_gravityForce.segment<3>(3) = Vector3d(0.0, 0.01 * -9.81, 0.0);
		m_gravityForce.segment<3>(6) = Vector3d(0.0, 0.01 * -9.81, 0.0);

		this->m_initialState = std::make_shared<MassPointsStateForStatic>();
	}

	const Vector& getExternalForces() const
	{
		return m_gravityForce;
	}

	Vector& computeF(const OdeState& state) override
	{
		// Internale deformation forces
		m_f = -computeK(state) * (state.getPositions() - m_initialState->getPositions());

		// Gravity pulling on the free nodes
		m_f += m_gravityForce;

		return m_f;
	}

	const Matrix& computeM(const OdeState& state) override
	{
		m_M.setZero();
		return m_M;
	}

	const Matrix& computeD(const OdeState& state) override
	{
		m_D.setZero();
		return m_D;
	}

	virtual const Matrix& computeK(const OdeState& state)
	{
		// A fake but valid stiffness matrix (node 0 fixed)
		m_K.setIdentity();
		m_K.block<6, 6>(3, 3).setConstant(2.0); // This adds coupling between nodes 1 and 2
		m_K.block<6, 6>(3, 3).diagonal().setConstant(10);
		return m_K;
	}

	void computeFMDK(const OdeState& state, Vector** f, Matrix** M, Matrix** D, Matrix** K) override
	{
		m_f = computeF(state);
		m_M = computeM(state);
		m_D = computeD(state);
		m_K = computeK(state);

		*f = &m_f;
		*K = &m_K;
		*D = &m_D;
		*M = &m_M;
	}

private:
	Vector m_f, m_gravityForce;
	Matrix m_M, m_D, m_K;
};

/// Class for the complex non-linear ODE a = x.v^2
class OdeComplexNonLinear : public OdeEquation
{
public:
	/// Constructor
	/// \param viscosity The mass viscosity
	OdeComplexNonLinear() : m_f(3), m_M(3, 3), m_D(3, 3), m_K(3, 3)
	{
		this->m_initialState = std::make_shared<MassPointState>();
	}

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	Vector& computeF(const OdeState& state) override
	{
		double v2 = state.getVelocities().dot(state.getVelocities());
		m_f = v2 * state.getPositions();
		return m_f;
	}

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	const Matrix& computeM(const OdeState& state) override
	{
		m_M.setIdentity();
		return m_M;
	}

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	const Matrix& computeD(const OdeState& state) override
	{
		m_D = 2.0 * state.getPositions() * state.getVelocities().transpose();
		return m_D;
	}

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	const Matrix& computeK(const OdeState& state)
	{
		m_K = Matrix::Identity(state.getNumDof(), state.getNumDof()) * state.getVelocities().squaredNorm();
		return m_K;
	}

	/// Evaluation of f(x,v), M(x,v), D = -df/dv(x,v), K = -df/dx(x,v)
	/// When all the terms are needed, this method can perform optimization in evaluating everything together
	/// \param state (x, v) the current position and velocity to evaluate the various terms with
	/// \param[out] f The RHS f(x,v)
	/// \param[out] M The matrix M(x,v)
	/// \param[out] D The matrix D = -df/dv(x,v)
	/// \param[out] K The matrix K = -df/dx(x,v)
	/// \note Returns pointers, the internal data will remain unchanged until the next call to computeFMDK() or
	/// \note computeF(), computeM(), computeD(), computeK()
	void computeFMDK(const OdeState& state, Vector** f, Matrix** M, Matrix** D, Matrix** K) override
	{
		computeF(state);
		computeM(state);
		computeD(state);
		computeK(state);

		*f = &m_f;
		*K = &m_K;
		*D = &m_D;
		*M = &m_M;
	}

	Vector m_f;
	Matrix m_M, m_D, m_K;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_UNITTESTS_MOCKOBJECT_H
