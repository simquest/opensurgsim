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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATIONFIXEDLOCALIZATION_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATIONFIXEDLOCALIZATION_H

#include <SurgSim/Physics/Localization.h>
#include <SurgSim/Physics/RigidRepresentation.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

/// This class implement the localization on a RigidRepresentation, as a local position
class RigidRepresentationFixedLocalization: public Localization
{
public:
	/// Default constructor
	RigidRepresentationFixedLocalization();

	/// Constructor
	/// \param representation The representation to assign to this localization
	explicit RigidRepresentationFixedLocalization(std::shared_ptr<Representation> representation);

	/// Destructor
	virtual ~RigidRepresentationFixedLocalization();

	/// Sets the local position
	/// \param p The local position to set the localization at
	void setLocalPosition(const SurgSim::Math::Vector3d& p);

	/// Gets the local position
	/// \return The local position set for this localization
	const SurgSim::Math::Vector3d& getLocalPosition() const;

private:
	/// Comparison function of equality
	/// \param localization The localization to compare it to
	/// \return True if the localization matches, False otherwise
	bool isEqual(const Localization& localization) const;

	/// Calculates the global position of this localization
	/// \param time The time in [0..1] at which the position should be calculated
	/// \return The global position of the localization at the requested time
	/// \note time can useful when dealing with CCD
	SurgSim::Math::Vector3d doCalculatePosition(double time);

	/// 3D position in local coordinates
	SurgSim::Math::Vector3d m_position;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_RIGIDREPRESENTATIONFIXEDLOCALIZATION_H
