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

	Vector& computeF(const OdeState& state) override
	{
		m_f = m_mass * m_gravity - m_viscosity * state.getVelocities();
		return m_f;
	}

	const SparseMatrix& computeM(const OdeState& state) override
	{
		m_M.setIdentity();
		m_M *= m_mass;
		return m_M;
	}

	const SparseMatrix& computeD(const OdeState& state) override
	{
		m_D.setIdentity();
		m_D *= m_viscosity;
		return m_D;
	}

	const SparseMatrix& computeK(const OdeState& state) override
	{
		m_K.setZero();
		return m_K;
	}

	void computeFMDK(const OdeState& state, Vector** f, SparseMatrix** M, SparseMatrix** D, SparseMatrix** K) override
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
	SparseMatrix m_M, m_D, m_K;
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
		// Internal deformation forces
		m_f = -computeK(state) * (state.getPositions() - m_initialState->getPositions());

		// Gravity pulling on the free nodes
		m_f += m_gravityForce;

		return m_f;
	}

	const SparseMatrix& computeM(const OdeState& state) override
	{
		m_M.setZero();
		return m_M;
	}

	const SparseMatrix& computeD(const OdeState& state) override
	{
		m_D.setZero();
		return m_D;
	}

	virtual const SparseMatrix& computeK(const OdeState& state) override
	{
		// A fake but valid stiffness matrix (node 0 fixed)
		// Desired matrix is:
		//
		// 1  0  0  0  0  0  0  0  0
		// 0  1  0  0  0  0  0  0  0
		// 0  0  1  0  0  0  0  0  0
		// 0  0  0 10  2  2  2  2  2
		// 0  0  0  2 10  2  2  2  2
		// 0  0  0  2  2 10  2  2  2
		// 0  0  0  2  2  2 10  2  2
		// 0  0  0  2  2  2  2 10  2
		// 0  0  0  2  2  2  2  2 10
		//
		// Generate it using sparse matrix notation.

		m_K.resize(9, 9);
		typedef Eigen::Triplet<double> T;
		std::vector<T> tripletList;
		tripletList.reserve(39);

		// Upper 3x3 identity block
		for (int counter = 0; counter < 3; ++counter)
		{
			tripletList.push_back(T(counter, counter, 1.0));
		}

		for (int row = 3; row < 9; ++row)
		{
			// Diagonal is 8 more than the rest of the 6x6 block
			tripletList.push_back(T(row, row, 8.0));

			// Now add in the 2's over the entire block
			for (int col = 3; col < 9; ++col)
			{
				tripletList.push_back(T(row, col, 2.0));
			}
		}
		m_K.setFromTriplets(tripletList.begin(), tripletList.end());
		std::cout << "m_K: " << std::endl << m_K << std::endl;
		return m_K;
	}

	void computeFMDK(const OdeState& state, Vector** f, SparseMatrix** M, SparseMatrix** D, SparseMatrix** K) override
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
	SparseMatrix m_M, m_D, m_K;
};

/// Class for the complex non-linear ODE a = x.v^2
class OdeComplexNonLinear : public OdeEquation
{
public:
	/// Constructor
	OdeComplexNonLinear() : m_f(3), m_M(3, 3), m_D(3, 3), m_K(3, 3)
	{
		this->m_initialState = std::make_shared<MassPointState>();
	}

	Vector& computeF(const OdeState& state) override
	{
		double v2 = state.getVelocities().dot(state.getVelocities());
		m_f = v2 * state.getPositions();
		return m_f;
	}

	const SparseMatrix& computeM(const OdeState& state) override
	{
		m_M.setIdentity();
		return m_M;
	}

	const SparseMatrix& computeD(const OdeState& state) override
	{
		auto position = 2.0 * state.getPositions();
		auto velocity = state.getVelocities();

		m_D.resize(state.getNumDof(), state.getNumDof());
		m_D.reserve(state.getNumDof());
		for (int row = 0; row < state.getNumDof(); ++row)
		{
			for (int col = 0; col < state.getNumDof(); ++col)
			{
				m_D.insert(row, col) = position[row] * velocity[col];
			}
		}

		return m_D;
	}

	const SparseMatrix& computeK(const OdeState& state) override
	{
		m_K.resize(state.getNumDof(), state.getNumDof());
		m_K.setIdentity();
		m_K *= state.getVelocities().squaredNorm();
		return m_K;
	}

	void computeFMDK(const OdeState& state, Vector** f, SparseMatrix** M, SparseMatrix** D, SparseMatrix** K) override
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

private:
	Vector m_f;
	SparseMatrix m_M, m_D, m_K;
};

}; // Math

}; // SurgSim

#endif // SURGSIM_MATH_UNITTESTS_MOCKOBJECT_H
