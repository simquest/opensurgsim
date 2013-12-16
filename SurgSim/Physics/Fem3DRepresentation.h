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

#ifndef SURGSIM_PHYSICS_FEM3DREPRESENTATION_H
#define SURGSIM_PHYSICS_FEM3DREPRESENTATION_H

#include <memory>

#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{

namespace Physics
{

/// Finite Element Model 3D is a fem built with 3D FemElement
class Fem3DRepresentation:
	public FemRepresentation<SurgSim::Math::Matrix, SurgSim::Math::Matrix, SurgSim::Math::Matrix, SurgSim::Math::Matrix>
{
public:
	/// Constructor
	/// \param name The name of the Fem3DRepresentation
	explicit Fem3DRepresentation(const std::string& name);

	/// Destructor
	virtual ~Fem3DRepresentation();

	/// Query the representation type
	/// \return the RepresentationType for this representation
	virtual RepresentationType getType() const override;

protected:
	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<DeformableRepresentationState> state,
		const SurgSim::Math::RigidTransform3d& transform) override;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DREPRESENTATION_H
