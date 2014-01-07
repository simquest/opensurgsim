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

#ifndef SURGSIM_PHYSICS_FEMREPRESENTATIONCOORDINATE_H
#define SURGSIM_PHYSICS_FEMREPRESENTATIONCOORDINATE_H

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

/// Structure which contains the elementId of an FemElement in an FemRepresentation and a coordinate in the FemElement.
struct FemRepresentationCoordinate {
	/// Default constructor with no initialization.
	FemRepresentationCoordinate();

	/// Constructor with initialization.
	/// \param elementId Numeric index of the FemElement contained in the FemRepresentation.
	/// \param naturalCoordinate Natural coordinates with respect to element.
	/// \note Constructor does not throw when given malformed parameters.
	FemRepresentationCoordinate(unsigned int elementId, SurgSim::Math::Vector naturalCoordinate);

	/// Numeric index of the FemElement contained in the FemRepresentation.
	unsigned int elementId;

	/// Barycentric Coordinate representing position with respect to the nodes of the FemElement.
	SurgSim::Math::Vector naturalCoordinate;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMREPRESENTATIONCOORDINATE_H
