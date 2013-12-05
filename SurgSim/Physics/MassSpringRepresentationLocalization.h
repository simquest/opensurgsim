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

#ifndef SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONLOCALIZATION_H
#define SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONLOCALIZATION_H

#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"

namespace SurgSim
{

namespace Physics
{

/// This class implements the localization of a MassSpringRepresentation, as a local position
class MassSpringRepresentationLocalization: public Localization
{
public:
	/// Default constructor
	MassSpringRepresentationLocalization();

	/// Constructor
	/// \param representation The representation to assign to this localization.
	explicit MassSpringRepresentationLocalization(std::shared_ptr<Representation> representation);

	/// Destructor
	virtual ~MassSpringRepresentationLocalization();

	/// Sets the local node.
	/// \param p The local node to set the localization at.
	void setLocalNode(size_t nodeID);

	/// Gets the local position.
	/// \return The local position set for this localization.
	const size_t& getLocalNode() const;

	/// Query if 'representation' is valid representation.
	/// \param	representation	The representation.
	/// \return	true if valid representation, false if not.
	virtual bool isValidRepresentation(std::shared_ptr<Representation> representation) override;

private:
	/// Calculates the global position of this localization.
	/// \param time The time in [0..1] at which the position should be calculated.
	/// \return The global position of the localization at the requested time.
	/// \note time can useful when dealing with CCD.
	virtual SurgSim::Math::Vector3d doCalculatePosition(double time) override;

	/// Node defining the localization.
	size_t m_nodeID;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONLOCALIZATION_H
