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

#ifndef SURGSIM_GRAPHICS_VECTORFIELD_H
#define SURGSIM_GRAPHICS_VECTORFIELD_H

#include <SurgSim/DataStructures/OptionalValue.h>
#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/Math/Vector.h>

#include <memory>

namespace SurgSim
{

namespace Graphics
{

/// A (mathematical) vector is represented as (X,Y,Z) associated with an optional color (R,G,B,alpha) information
struct VectorFieldData
{
	/// Direction (X,Y,Z) of the vector
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d> vectorDirection;
	/// Color (R,G,B,alpha) of the vector
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector4d> vectorColor;

	/// Compare the vectors and return true if equal, false if not equal.
	/// \return True if vertex1 and vertex2 have the same value; Otherwise, false.
	friend bool operator==(const VectorFieldData& vertex1, const VectorFieldData& vertex2)
	{
		return vertex1.vectorDirection.hasValue() && vertex2.vectorDirection.hasValue() &&
			   vertex1.vectorColor.hasValue() && vertex2.vectorColor.hasValue() &&
			   vertex1.vectorDirection.getValue() == vertex2.vectorDirection.getValue() &&
			   vertex1.vectorColor.getValue() == vertex2.vectorColor.getValue();
	}

	/// Compare the vectors and return true if not equal, false if equal.
	/// \return True if vertex1 and vertex2 have different values; Otherwise, false.
	friend bool operator!=(const VectorFieldData& vertex1, const VectorFieldData& vertex2)
	{
		return ! (vertex1 == vertex2);
	}
};

typedef SurgSim::DataStructures::Vertices<VectorFieldData> VectorField;

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_VECTORFIELD_H