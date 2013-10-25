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

#ifndef SURGSIM_DATASTRUCTURES_VECTOR_H
#define SURGSIM_DATASTRUCTURES_VECTOR_H

#include <SurgSim/Math/Vector.h>

#include <memory>

namespace SurgSim
{

namespace DataStructures
{

/// A (mathematical) vector is represented as (X,Y,Z) associated with an optional color (R,G,B,alpha) information
/// The vector is set to white by default
struct Vector
{
	explicit Vector(const SurgSim::Math::Vector3d& vec,
					const SurgSim::Math::Vector4d& col = SurgSim::Math::Vector4d(1.0, 1.0, 1.0, 1.0)) :
					vector(vec), color(col)
	{
	}

	/// Compare the vectors and return true if equal, false if not equal.
	friend bool operator==(const Vector& vertex1, const Vector& vertex2)
	{
		return vertex1.vector == vertex2.vector && vertex1.color == vertex2.color;
	}

	/// Compare the vectors and return false if equal, true if not equal.
	friend bool operator!=(const Vector& vertex1, const Vector& vertex2)
	{
		return ! (vertex1 == vertex2);
	}

	/// vector (X, Y, Z)
	SurgSim::Math::Vector3d vector;
	/// color (R,G,B, alpha)
	SurgSim::Math::Vector4d color;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_VECTOR_H