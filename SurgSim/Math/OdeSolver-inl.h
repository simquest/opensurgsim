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

template <class State>
OdeSolver<State>::OdeSolver(OdeEquation<State>* equation)
	: m_equation(*equation)
{
	allocate(m_equation.getInitialState()->getPositions().size());

	// Default linear solver
	setLinearSolver(std::make_shared<LinearSolveAndInverseDenseMatrix>());
}

template <class State>
const std::string OdeSolver<State>::getName() const
{
	return m_name;
}

template <class State>
void OdeSolver<State>::setLinearSolver(std::shared_ptr<LinearSolveAndInverse> linearSolver)
{
	m_linearSolver = linearSolver;
}

template <class State>
std::shared_ptr<LinearSolveAndInverse> OdeSolver<State>::getLinearSolver() const
{
	return m_linearSolver;
}

template <class State>
const Matrix& OdeSolver<State>::getSystemMatrix() const
{
	return m_systemMatrix;
}

template <class State>
const Matrix& OdeSolver<State>::getCompliance() const
{
	return m_compliance;
}

template <class State>
void OdeSolver<State>::allocate(unsigned int size)
{
	resizeMatrix(&m_systemMatrix, size, size);
	resizeMatrix(&m_compliance, size, size);
}

}; // namespace Math

}; // namespace SurgSim

#endif // SURGSIM_MATH_ODESOLVER_INL_H
