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

#ifndef SURGSIM_PHYSICS_FEM3DCOROTATIONALTETRAGEDRONREPRESENTATION_H
#define SURGSIM_PHYSICS_FEM3DCOROTATIONALTETRAGEDRONREPRESENTATION_H

#include <memory>
#include <string>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

namespace SurgSim
{

namespace Physics
{
SURGSIM_STATIC_REGISTRATION(Fem3DCorotationalTetrahedronRepresentation);

/// Co-rotational Tetrahedron Finite Element Model 3D is a fem built with co-rotational tetrahedron 3D FemElement
/// It derives from Fem3DRepresentation from which it uses most functionalities.
/// The only difference comes in the initialization and update to take a special care of
/// the FemElement's rotation.
class Fem3DCorotationalTetrahedronRepresentation : public SurgSim::Physics::Fem3DRepresentation
{
public:
	/// Constructor
	/// \param name The name of the Fem3DCorotationalTetrahedronRepresentation
	explicit Fem3DCorotationalTetrahedronRepresentation(const std::string& name);

	/// Destructor
	virtual ~Fem3DCorotationalTetrahedronRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::Fem3DCorotationalTetrahedronRepresentation);

	void setFemElementType(const std::string& type) override;

protected:
	SurgSim::Math::Matrix getNodeTransformation(const SurgSim::Math::OdeState& state, size_t nodeId) override;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DCOROTATIONALTETRAGEDRONREPRESENTATION_H
