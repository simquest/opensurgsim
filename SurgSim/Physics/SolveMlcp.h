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

#ifndef SURGSIM_PHYSICS_SOLVEMLCP_H
#define SURGSIM_PHYSICS_SOLVEMLCP_H

#include <memory>

#include "SurgSim/Math/MlcpGaussSeidelSolver.h"
#include "SurgSim/Physics/Computation.h"

namespace SurgSim
{
namespace Physics
{

/// Solve the system Mixed Linear Complementarity Problem (Mlcp)
class SolveMlcp : public Computation
{
public:
	/// Constructor
	/// \param doCopyState Specify if the ouput state is a copy or not of the input state in Computation::Update()
	explicit SolveMlcp(bool doCopyState = false);

	/// Destructor
	virtual ~SolveMlcp();

	/// Set the maximum number of iterations for the MLCP solver.
	/// \param maxIterations The maximum number of iterations.
	void setMaxIterations(size_t maxIterations);

	/// Get the maximum number of iterations for the MLCP solver.
	/// \return The maximum number of iterations.
	size_t getMaxIterations() const;

	/// Set the precision of the MLCP solver.
	/// \param epsilon The precision.
	void setPrecision(double epsilon);

	/// Get the precision of the MLCP solver.
	/// \return The precision.
	double getPrecision() const;

	/// Set the contact tolerance for the MLCP solver.
	/// \param epsilon The contact tolerance.
	void setContactTolerance(double epsilon);

	/// Get the contact tolerance for the MLCP solver.
	/// \return The contact tolerance.
	double getContactTolerance() const;

protected:

	/// Override doUpdate from superclass
	/// \param dt The time step
	/// \param state The Physics manager state
	/// \return The updated physics manager state (input updated or copied updated)
	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt, const std::shared_ptr<PhysicsManagerState>& state)
		override;

private:

	/// The Gauss-Seidel Mlcp solver
	SurgSim::Math::MlcpGaussSeidelSolver m_gaussSeidelSolver;
};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_SOLVEMLCP_H
