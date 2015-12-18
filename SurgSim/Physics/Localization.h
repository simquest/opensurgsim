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

#ifndef SURGSIM_PHYSICS_LOCALIZATION_H
#define SURGSIM_PHYSICS_LOCALIZATION_H

#include <memory>

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{

namespace DataStructures
{
struct Location;
}

namespace Physics
{

class Representation;

/// This class localize a point on a representation (representation specific)
class Localization
{
public:
	/// Default constructor
	Localization();

	/// Constructor
	/// \param representation The representation on which the localization is defined
	explicit Localization(std::shared_ptr<Representation> representation);

	/// Destructor
	virtual ~Localization();

	/// Sets the representation
	/// \param representation The representation on which the localization is defined
	void setRepresentation(std::shared_ptr<Representation> representation);

	/// Gets the representation
	/// \return The representation on which the localization is defined, nullptr if none has been defined
	std::shared_ptr<Representation> getRepresentation() const;

	/// Calculates the global position of this localization
	/// \param time The time in [0..1] at which the position should be calculated
	/// \return The global position of the localization at the requested time
	/// \note time can useful when dealing with CCD
	SurgSim::Math::Vector3d calculatePosition(double time = 1.0);

	virtual bool isValidRepresentation(std::shared_ptr<Representation> representation);

	/// \return The rigid transform of the localization.
	virtual Math::RigidTransform3d getTransform();

	/// \param point Move this localization closest to this point
	/// \param hasReachedEnd [out] Flag to set, when the localization reaches the end of the representation.
	/// \return Whether the localization was moved or not.
	virtual bool moveClosestTo(const Math::Vector3d& point, bool *hasReachedEnd);

private:
	/// Calculates the global position of this localization
	/// \param time The time in [0..1] at which the position should be calculated
	/// \return The global position of the localization at the requested time
	/// \note time can useful when dealing with CCD
	virtual SurgSim::Math::Vector3d doCalculatePosition(double time) = 0;

	/// The representation on which the localization is defined
	std::shared_ptr<Representation> m_representation;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_LOCALIZATION_H
