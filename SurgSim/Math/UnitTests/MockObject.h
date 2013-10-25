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

#include <SurgSim/Math/OdeEquation.h>

namespace SurgSim
{

namespace Math
{

class MassPointState
{
public:
	MassPointState()
	{
		m_x.resize(3); m_x.setZero();
		m_v.resize(3); m_v.setZero();
		m_a.resize(3); m_a.setZero();
	}

	const Vector& getPositions() const { return m_x; }
	Vector& getPositions() { return m_x; }

	const Vector& getVelocities() const { return m_v; }
	Vector& getVelocities() { return m_v; }

	const Vector& getAccelerations() const { return m_a; }
	Vector& getAccelerations() { return m_a; }

	bool operator ==(const MassPointState& state) const
	{
		return m_x.isApprox(state.m_x) && m_v.isApprox(state.m_v) && m_a.isApprox(state.m_a);
	}

	bool operator !=(const MassPointState& state) const
	{
		return !((*this) == state);
	}

private:
	Vector m_x, m_v, m_a;
};

class MassPoint : public SurgSim::Math::OdeEquation<MassPointState, Matrix, Matrix, Matrix, Matrix>
{
public:
	/// Constructor
	/// \param x The mass initial position
	/// \param mass The mass (in Kg)
	MassPoint() : m_mass(1.0), m_gravity(0.0, -9.81, 0.0), m_f(3), m_M(3, 3), m_D(3, 3), m_K(3, 3)
	{
		this->m_initialState = std::make_shared<MassPointState>();
	}

	/// Evaluation of the RHS function f(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the function f(x,v) with
	/// \return The vector containing f(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeF() or computeFMDK()
	Vector& computeF(const MassPointState& state) override
	{
		m_f = m_mass * m_gravity;
		return m_f;
	}

	/// Evaluation of the LHS matrix M(x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the matrix M(x,v) with
	/// \return The matrix M(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeM() or computeFMDK()
	const Matrix& computeM(const MassPointState& state) override
	{
		m_M.setIdentity();
		m_M *= m_mass;
		return m_M;
	}

	/// Evaluation of D = -df/dv (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix D = -df/dv(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeD() or computeFMDK()
	const Matrix& computeD(const MassPointState& state) override
	{
		m_D.setZero();
		return m_D;
	}

	/// Evaluation of K = -df/dx (x,v) for a given state
	/// \param state (x, v) the current position and velocity to evaluate the Jacobian matrix with
	/// \return The matrix K = -df/dx(x,v)
	/// \note Returns a reference, its values will remain unchanged until the next call to computeK() or computeFMDK()
	const Matrix& computeK(const MassPointState& state)
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
	void computeFMDK(const MassPointState& state, Vector** f, Matrix** M, Matrix** D, Matrix** K) override
	{
		m_M.setIdentity();
		m_M *= m_mass;
		m_D.setZero();
		m_K.setZero();
		m_f = m_mass * m_gravity;

		*f = &m_f;
		*K = &m_K;
		*D = &m_D;
		*M = &m_M;
	}

	double getEnergy(const MassPointState& s) const
	{
		double kineticEnergy = 0.5 * m_mass * s.getVelocities().dot(s.getVelocities());
		double potentialEnergy = 0.5 * m_mass * m_gravity.norm() * s.getPositions()[1];
		return kineticEnergy + potentialEnergy;
	}

	double m_mass;
	Vector3d m_gravity;
	Vector m_f;
	Matrix m_M, m_D, m_K;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_UNITTESTS_MOCKOBJECT_H
