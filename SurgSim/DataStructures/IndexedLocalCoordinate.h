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

#ifndef SURGSIM_DATASTRUCTURES_INDEXEDLOCALCOORDINATE_H
#define SURGSIM_DATASTRUCTURES_INDEXEDLOCALCOORDINATE_H

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace DataStructures
{

/// Structure which contains the elementId of an FemElement in an FemRepresentation and a coordinate in the FemElement.
struct IndexedLocalCoordinate
{
	/// Default constructor with no initialization.
	IndexedLocalCoordinate();

	/// Constructor with initialization.
	/// \param elementId Numeric index.
	/// \param barycentricCoordinate Barycentric coordinates with respect to element.
	/// \note Constructor does not throw when given malformed parameters.
	IndexedLocalCoordinate(size_t elementId, const SurgSim::Math::Vector& barycentricCoordinate);

	/// Numeric index to indicate the entity w.r.t which the barycentricCoordinate is defined.
	size_t elementId;

	/// Barycentric Coordinate representing position with respect to the nodes of the MeshElement.
	SurgSim::Math::Vector barycentricCoordinate;
};

} // namespace DataStructures

} // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_INDEXEDLOCALCOORDINATE_H
