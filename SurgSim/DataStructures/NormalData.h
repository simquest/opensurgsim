// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_NORMALDATA_H
#define SURGSIM_DATASTRUCTURES_NORMALDATA_H

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

/// Store normal for each triangle in a triangle mesh.
struct NormalData
{
	SurgSim::Math::Vector3d normal;

	/// Equality operator.
	/// \param	rhs	The right hand side NormalData.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const NormalData& rhs) const
	{
		return normal == rhs.normal;
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side NormalData.
	/// \return	true if the parameters are not considered equivalent.
	bool operator!=(const NormalData& rhs) const
	{
		return !((*this) == rhs);
	}
};

};
};

#endif // SURGSIM_DATASTRUCTURES_NORMALDATA_H
