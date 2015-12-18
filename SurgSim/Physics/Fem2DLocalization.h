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

#ifndef SURGSIM_PHYSICS_FEM2DLOCALIZATION_H
#define SURGSIM_PHYSICS_FEM2DLOCALIZATION_H

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Physics/FemLocalization.h"

namespace SurgSim
{

namespace Physics
{

/// Implementation of Localization for Fem2DRepresentation
///
/// Fem2DLocalization tracks the global coordinates of an IndexedLocalCoordinate associated with an
/// Fem2DRepresentation. The IndexedLocalCoordinate must be related to an FemElement (the index is an FemElement id and
/// the local coordinates are the barycentric coordinates of the nodes in this FemElement).
class Fem2DLocalization : public FemLocalization
{
public:
	/// Constructor
	/// \param representation The representation to assign to this localization.
	/// \param localCoordinate The indexed local coordinate relative to the representation.
	Fem2DLocalization(std::shared_ptr<Representation> representation,
									const SurgSim::DataStructures::IndexedLocalCoordinate& localCoordinate);

	/// Destructor
	virtual ~Fem2DLocalization();

	/// Query if 'representation' is valid representation.
	/// \param	representation	The representation.
	/// \return	true if valid representation, false if not.
	bool isValidRepresentation(std::shared_ptr<Representation> representation) override;

	Math::RigidTransform3d getTransform() override;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM2DLOCALIZATION_H
