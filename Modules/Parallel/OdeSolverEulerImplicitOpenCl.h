// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PARALLEL_ODESOLVEREULERIMPLICITOPENCL_H
#define SURGSIM_PARALLEL_ODESOLVEREULERIMPLICITOPENCL_H

#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/OdeState.h"

namespace SurgSim
{
namespace Parallel
{
class OdeSolverEulerImplicitOpenCl : public Math::OdeSolver
{
public:
	OdeSolverEulerImplicitOpenCl(Math::OdeEquation* odeEquation);


	virtual void solve(double dt,
					   const Math::OdeState& currentState,
					   Math::OdeState* newState,
					   bool computeCompliance = true) override;

	void setNewtonRaphsonMaximumIteration(size_t val)
	{

	}

	void setNewtonRaphsonEpsilonConvergence(double e)
	{

	}
protected:
	virtual void assembleLinearSystem(double dt,
									  const Math::OdeState& state,
									  const Math::OdeState& newState,
									  bool computeRHS = true) override;

};


}
}

#endif