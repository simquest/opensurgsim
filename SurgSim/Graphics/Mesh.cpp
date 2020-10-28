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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/MeshPlyReaderDelegate.h"

using SurgSim::DataStructures::EmptyData;


template<>
std::string SurgSim::DataStructures::TriangleMesh<SurgSim::Graphics::VertexData, EmptyData, EmptyData>
::m_className = "SurgSim::Graphics::Mesh";

namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Graphics::Mesh, Mesh);

Mesh::Mesh() :
	m_updateCount(1)
{
}

Mesh::Mesh( const Mesh& other ) : BaseType(other), m_updateCount(other.getUpdateCount())
{
}

Mesh::Mesh( Mesh&& other ) : BaseType(std::move(other)), m_updateCount(std::move(other.m_updateCount))
{
}

void Mesh::initialize(
	const std::vector<SurgSim::Math::Vector3d>& vertices,
	const std::vector<SurgSim::Math::Vector4d>& colors,
	const std::vector<SurgSim::Math::Vector2d>& textures,
	const std::vector<size_t>& triangles)
{
	SURGSIM_ASSERT(textures.empty() || textures.size() >= vertices.size()) <<
			"To make a mesh you need to either provide at least the same amount" <<
			" of texture coordinates as vertices or none at all.";
	SURGSIM_ASSERT(colors.empty() || colors.size() >= vertices.size()) <<
			"To make a mesh you need to either provide at least the same amount" <<
			" of colors as vertices or none at all.";

	clear();

	size_t i = 0;
	for (auto it = std::begin(vertices); it != std::end(vertices); ++it, ++i)
	{
		VertexData data;
		if (! colors.empty())
		{
			data.color.setValue(colors[i]);
		}
		if (! textures.empty())
		{
			data.texture.setValue(textures[i]);
		}
		addVertex(Mesh::VertexType(*it, data));
	}

	for (size_t i = 0; i < triangles.size(); i += 3)
	{
		TriangleType::IdType ids = {{triangles[i], triangles[i + 1], triangles[i + 2]}};

		bool valid = true;
		for (auto it = std::begin(ids); it != std::end(ids); ++it)
		{
			if (*it >= getNumVertices())
			{
				valid = false;
				break;
			}
		}

		if (valid)
		{
			Mesh::TriangleType triangle(ids);
			addTriangle(triangle);
		}
		else
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
					"When building a mesh a vertex was present in a triangle that was not in the list of vertices";
		}
	}
}

bool Mesh::doLoad(const std::string& fileName)
{
	SurgSim::DataStructures::PlyReader reader(fileName);
	if (! reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
				<< "'" << fileName << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<MeshPlyReaderDelegate>(std::dynamic_pointer_cast<Mesh>(shared_from_this()));
	if (! reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
				<< "The input file '" << fileName << "' does not have the property required by triangle mesh.";
		return false;
	}

	return true;
}

void Mesh::dirty()
{
	++m_updateCount;
}

size_t Mesh::getUpdateCount() const
{
	return m_updateCount;
}

Mesh& Mesh::operator=( const Mesh& other )
{
	BaseType::operator=(other);
	return *this;
}

Mesh& Mesh::operator=( Mesh&& other )
{
	BaseType::operator=(std::move(other));
	return *this;
}

}; // Graphics
}; // SurgSim
