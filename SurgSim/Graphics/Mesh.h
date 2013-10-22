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

#ifndef SURGSIM_GRAPHICS_MESH_H
#define SURGSIM_GRAPHICS_MESH_H

#include <SurgSim/DataStructures/TriangleMesh.h>
#include <SurgSim/DataStructures/OptionalValue.h>


namespace SurgSim
{

namespace Graphics
{

class VertexData {

public:

	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector2d> texture;
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector4d> color;
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d> normal;

	/// Equality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const SurgSim::Graphics::VertexData& rhs) const
	{
		return texture == rhs.texture &&
			   color == rhs.color &&
			   normal == rhs.normal;
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are not considered equivalent.
	bool operator !=(const SurgSim::Graphics::VertexData& rhs) const
	{
		return !((*this) == rhs);
	}
};

class TriangleData {
public:
	SurgSim::Math::Vector3d normal;

	/// Equality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const SurgSim::Graphics::TriangleData& rhs) const
	{
		return normal == rhs.normal;
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are not considered equivalent.
	bool operator !=(const SurgSim::Graphics::TriangleData& rhs) const
	{
		return !((*this) == rhs);
	}
};

typedef SurgSim::DataStructures::TriangleMesh<VertexData, void, TriangleData> Mesh;

}; // Graphics
}; // SurgSim

#endif