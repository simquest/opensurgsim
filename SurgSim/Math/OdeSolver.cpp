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

#include <SurgSim/Math/OdeSolver.h>

namespace SurgSim
{

namespace Math
{

template <class State, class MType, class DType, class KType, class SType>
OdeSolver<State, MType, DType, KType, SType>::OdeSolver(const OdeEquation& equation, const State& initialState) :
	m_equation(equation),
	m_initialState(initialState),
	m_constantMinitialized(false),
	m_constantDinitialized(false),
	m_constantKinitialized(false)
{
	allocate(initialState.getPositions().size());
}
	
template <class State, class MType, class DType, class KType, class SType>
const MType& OdeSolver<State, MType, DType, KType, SType>::getM() const
{
	return m_M;
}

template <class State, class MType, class DType, class KType, class SType>
const DType& OdeSolver<State, MType, DType, KType, SType>::getD() const
{
	return m_D;
}

template <class State, class MType, class DType, class KType, class SType>
const KType& OdeSolver<State, MType, DType, KType, SType>::getK() const
{
	return m_K;
}

template <class State, class MType, class DType, class KType, class SType>
const SType& OdeSolver<State, MType, DType, KType, SType>::getSystemMatrix() const
{
	return m_systemMatrix;
}

template <class State, class MType, class DType, class KType, class SType>
const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign>& OdeSolver<State, MType, DType, KType, SType>::getCompliance() const
{
	return m_compliance;
}

template <class State, class MType, class DType, class KType, class SType>
const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& OdeSolver<State, MType, DType, KType, SType>::getF() const
{
	return m_f;
}

template <class State, class MType, class DType, class KType, class SType>
const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>& OdeSolver<State, MType, DType, KType, SType>::getA() const
{
	return m_a;
}

template <class State, class MType, class DType, class KType, class SType>
void OdeSolver<State, MType, DType, KType, SType>::allocate(unsigned int size)
{
	m_equation.resizeMatricesToFitState(m_M, m_D, m_K, m_systemMatrix, size);
	m_equation.resizeVectorToFitState(m_a, size);
	m_equation.resizeVectorToFitState(m_f, size);
	m_compliance.resize(size, size);
}

template <class State, class MType, class DType, class KType, class SType>
void OdeSolver<State, MType, DType, KType, SType>::computeF(const State& currentState)
{
	m_equation.computeF(currentState, &m_f);
}

template <class State, class MType, class DType, class KType, class SType>
void OdeSolver<State, MType, DType, KType, SType>::computeM(const State& currentState)
{
	if (m_equation.isMConstant())
	{
		if (! m_constantMinitialized)
		{
			m_equation.computeM(currentState, &m_M);
			m_constantMinitialized = true;
		}
	}
	else
	{
		m_equation.computeM(currentState, &m_M);
	}
}

template <class State, class MType, class DType, class KType, class SType>
void OdeSolver<State, MType, DType, KType, SType>::computeD(const State& currentState)
{
	if (m_equation.isDConstant())
	{
		if (! m_constantDinitialized)
		{
			m_equation.computeD(currentState, &m_D);
			m_constantDinitialized = true;
		}
	}
	else
	{
		m_equation.computeD(currentState, &m_D);
	}
}

template <class State, class MType, class DType, class KType, class SType>
void OdeSolver<State, MType, DType, KType, SType>::computeK(const State& currentState)
{
	if (m_equation.isKConstant())
	{
		if (! m_constantKinitialized)
		{
			m_equation.computeK(currentState, &m_K);
			m_constantKinitialized = true;
		}
	}
	else
	{
		m_equation.computeK(currentState, &m_K);
	}
}


}; // namespace Math

}; // namespace SurgSim
