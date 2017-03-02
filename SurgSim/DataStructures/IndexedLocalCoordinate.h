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

/// A generic (size_t index, Vector coordinate) pair. The coordinate is a dynamic size vector.
/// E.g. This can be used to represent a barycentric coordinate within a simplex (identified by the index).
struct IndexedLocalCoordinate
{
	/// Default constructor with no initialization.
	IndexedLocalCoordinate();

	/// Constructor with initialization.
	/// \param index Numeric index.
	/// \param coordinate Coordinates with respect to the entity identified by the index.
	/// \note Constructor does not throw when given malformed parameters.
	IndexedLocalCoordinate(size_t index, const SurgSim::Math::Vector& coordinate);

	/// Numeric index to indicate the entity w.r.t which the barycentricCoordinate is defined.
	size_t index;

	/// Coordinates with respect to the entity identified by the index.
	SurgSim::Math::Vector coordinate;

	/// Comparison method 'isApprox'
	/// \param other The other IndexedLocalCoordinate to compare it to
	/// \param precision The precision with which to compare
	/// \return True if the two IndexedLocalCoordinate are equal within precision, False otherwise
	bool isApprox(const IndexedLocalCoordinate& other, double precision = std::numeric_limits<double>::epsilon())
		const;

	bool isValid()
	{
		return (std::abs(coordinate.sum() - 1.0) < 1e-10) && (-1e-10 <= coordinate.minCoeff() &&
			coordinate.maxCoeff() <= 1.0 + 1e-10);
	}
};

} // namespace DataStructures

} // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_INDEXEDLOCALCOORDINATE_H
