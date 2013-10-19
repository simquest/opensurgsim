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

#ifndef SURGSIM_MATH_ODESOLVER_INL_H
#define SURGSIM_MATH_ODESOLVER_INL_H

namespace SurgSim
{

namespace Math
{

template <class State, class MT, class DT, class KT, class ST>
OdeSolver<State, MT, DT, KT, ST>::OdeSolver(OdeEquation<State, MT, DT, KT, ST>& equation) :
	m_equation(equation)
{
	allocate(equation.getInitialState()->getPositions().size());
}

template <class State, class MT, class DT, class KT, class ST>
const std::string OdeSolver<State, MT, DT, KT, ST>::getName() const
{
	return m_name;
}

template <class State, class MT, class DT, class KT, class ST>
const ST& OdeSolver<State, MT, DT, KT, ST>::getSystemMatrix() const
{
	return m_systemMatrix;
}

template <class State, class MT, class DT, class KT, class ST>
const Matrix& OdeSolver<State, MT, DT, KT, ST>::getCompliance() const
{
	return m_compliance;
}

template <class State, class MT, class DT, class KT, class ST>
void OdeSolver<State, MT, DT, KT, ST>::allocate(unsigned int size)
{
	resize(&m_systemMatrix, size, size);
	resize(&m_compliance, size, size);
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVER_INL_H
