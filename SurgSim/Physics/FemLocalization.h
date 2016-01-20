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

#ifndef SURGSIM_PHYSICS_FEMLOCALIZATION_H
#define SURGSIM_PHYSICS_FEMLOCALIZATION_H

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Physics/Localization.h"

namespace SurgSim
{

namespace Physics
{

/// Implementation of Localization for Fem3DRepresentation
///
/// FemLocalization tracks the global coordinates of an IndexedLocalCoordinate associated with an
/// FemRepresentation. The IndexedLocalCoordinate must be related to an FemElement (the index is an FemElement id and
/// the local coordinates are the barycentric coordinates of the nodes in this FemElement).
/// It is used, for example, as a helper class for filling out the MlcpPhysicsProblem in
/// Fem3DRepresentationContact::doBuild, which constrains the motion of Fem3DRepresentation at a frictionless contact.
class FemLocalization : public Localization
{
public:
	/// Constructor
	/// \param representation The representation to assign to this localization.
	/// \param localPosition The local position to set the localization at.
	FemLocalization(std::shared_ptr<Representation> representation,
		const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition);

	/// Destructor
	virtual ~FemLocalization();

	/// Sets the local position.
	/// \param localPosition The local position to set the localization at.
	void setLocalPosition(const SurgSim::DataStructures::IndexedLocalCoordinate& localPosition);

	/// Gets the local position.
	/// \return The local position set for this localization.
	const SurgSim::DataStructures::IndexedLocalCoordinate& getLocalPosition() const;

private:
	SurgSim::Math::Vector3d doCalculatePosition(double time) override;

	SurgSim::Math::Vector3d doCalculateVelocity(double time) override;

	/// Barycentric position in local coordinates
	SurgSim::DataStructures::IndexedLocalCoordinate m_position;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMLOCALIZATION_H
