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

#ifndef SURGSIM_PHYSICS_FEM1DLOCALIZATION_H
#define SURGSIM_PHYSICS_FEM1DLOCALIZATION_H

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Physics/Localization.h"

namespace SurgSim
{

namespace Physics
{

/// Implementation of Localization for Fem1DRepresentation
///
/// Fem1DLocalization tracks the global coordinates of an IndexedLocalCoordinate associated with an
/// Fem1DRepresentation. The IndexedLocalCoordinate must be related to an FemElement (the index is an FemElement id and
/// the local coordinates are the barycentric coordinates of the nodes in this FemElement).
class Fem1DLocalization : public Localization
{
public:
	/// Constructor
	/// \param representation The representation to assign to this localization.
	/// \param localCoordinate The indexed local coordinate relative to the representation.
	Fem1DLocalization(std::shared_ptr<Representation> representation,
									const SurgSim::DataStructures::IndexedLocalCoordinate& localCoordinate);

	/// Destructor
	virtual ~Fem1DLocalization();

	/// Sets the local position.
	/// \param localCoordinate The local position to set the localization at.
	void setLocalPosition(const SurgSim::DataStructures::IndexedLocalCoordinate& localCoordinate);

	/// Gets the local position.
	/// \return The local position set for this localization.
	const SurgSim::DataStructures::IndexedLocalCoordinate& getLocalPosition() const;

	/// Query if 'representation' is valid representation.
	/// \param	representation	The representation.
	/// \return	true if valid representation, false if not.
	bool isValidRepresentation(std::shared_ptr<Representation> representation) override;

private:
	/// Calculates the global position of this localization.
	/// \param time The time in [0..1] at which the position should be calculated.
	/// \return The global position of the localization at the requested time.
	/// \note time can be useful when dealing with CCD.
	SurgSim::Math::Vector3d doCalculatePosition(double time);

	/// Barycentric position in local coordinates
	SurgSim::DataStructures::IndexedLocalCoordinate m_position;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM1DLOCALIZATION_H
