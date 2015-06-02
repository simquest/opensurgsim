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

#ifndef SURGSIM_PHYSICS_PERFORMANCETESTS_MOCKFEM3DREPRESENTATION_H
#define SURGSIM_PHYSICS_PERFORMANCETESTS_MOCKFEM3DREPRESENTATION_H

#include "SurgSim/Physics/Fem3DRepresentation.h"

namespace SurgSim
{
namespace Physics
{

/// Testing class used to publicly expose Fem3DRepresentation's protected ODE solver
class MockFem3DRepresentation : public SurgSim::Physics::Fem3DRepresentation
{
public:
	/// Constructor:
	/// \param name Name of the Fem3DRepresentation
	explicit MockFem3DRepresentation(const std::string& name)
		: Fem3DRepresentation(name)
	{
	}

	/// \return the ODE solver being used
	std::shared_ptr<SurgSim::Math::OdeSolver> getOdeSolver()
	{
		return m_odeSolver;
	}
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_PERFORMANCETESTS_MOCKFEM3DREPRESENTATION_H
