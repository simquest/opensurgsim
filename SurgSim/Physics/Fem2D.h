// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_FEM2D_H
#define SURGSIM_PHYSICS_FEM2D_H

#include "SurgSim/Physics/Fem.h"

namespace SurgSim
{
namespace Physics
{

SURGSIM_STATIC_REGISTRATION(Fem2D);

/// Fem class data structure implementation for 2-Dimensional FEMs
/// \sa Fem
class Fem2D : public Fem<FemElementStructs::RotationVectorData, FemElementStructs::FemElement2DParameter>
{
public:
	Fem2D();

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem2D);

protected:
	// Asset API override
	bool doLoad(const std::string& filePath) override;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM2D_H
