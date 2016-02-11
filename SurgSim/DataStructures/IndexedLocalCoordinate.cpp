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

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"

namespace SurgSim
{

namespace DataStructures
{

IndexedLocalCoordinate::IndexedLocalCoordinate() : index(0)
{
}

IndexedLocalCoordinate::IndexedLocalCoordinate(size_t index, const SurgSim::Math::Vector& coordinate)
	: index(index), coordinate(coordinate)
{
}

bool IndexedLocalCoordinate::isApprox(const IndexedLocalCoordinate& other, double precision) const
{
	return (index == other.index) &&
		((coordinate.isZero(precision) && other.coordinate.isZero(precision)) ||
			coordinate.isApprox(other.coordinate, precision));
}

} // namespace DataStructures

} // namespace SurgSim
