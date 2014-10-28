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

#include <memory>

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::DataStructures::TriangleMesh, TriangleMesh);

namespace SurgSim
{
namespace DataStructures
{

TriangleMesh::TriangleMesh()
{
}

const SurgSim::Math::Vector3d& TriangleMesh::getNormal(size_t triangleId)
{
	return getTriangle(triangleId).data.normal;
}

void TriangleMesh::calculateNormals()
{
	for (size_t i = 0; i < getNumTriangles(); ++i)
	{
		const SurgSim::Math::Vector3d& vertex0 = getVertexPosition(getTriangle(i).verticesId[0]);
		const SurgSim::Math::Vector3d& vertex1 = getVertexPosition(getTriangle(i).verticesId[1]);
		const SurgSim::Math::Vector3d& vertex2 = getVertexPosition(getTriangle(i).verticesId[2]);

		// Calculate normal vector
		SurgSim::Math::Vector3d normal = (vertex1 - vertex0).cross(vertex2 - vertex0);
		normal.normalize();

		getTriangle(i).data.normal = normal;
	}
}

void TriangleMesh::doUpdate()
{
	calculateNormals();
}

bool TriangleMesh::doLoad(const std::string& fileName)
{
	auto triangleMeshDelegate = std::make_shared<TriangleMeshPlyReaderDelegate<TriangleMesh>>(shared_from_this());

	PlyReader reader(fileName);
	SURGSIM_ASSERT(reader.isValid()) << "'" << fileName << "' is an invalid .ply file.";
	SURGSIM_ASSERT(reader.parseWithDelegate(triangleMeshDelegate)) <<
			"The input file " << fileName << " does not have the property required by triangle mesh.";

	calculateNormals();

	return true;
}

void TriangleMesh::copyWithTransform(const SurgSim::Math::RigidTransform3d& pose, const TriangleMesh& source)
{
	SURGSIM_ASSERT(getNumVertices() == source.getNumVertices())
			<< "The source mesh must have the same number of vertices.";
	SURGSIM_ASSERT(getNumEdges() == source.getNumEdges())
			<< "The source mesh must have the same number of edges";
	SURGSIM_ASSERT(getNumTriangles() == source.getNumTriangles())
			<< "The source mesh must have the same number of triangles";

	auto targetVertex = getVertices().begin();
	auto const& vertices = source.getVertices();
	for (auto it = vertices.cbegin(); it != vertices.cend(); ++it)
	{
		targetVertex->position = pose * it->position;
		++targetVertex;
	}

	auto targetTriangle = getTriangles().begin();
	auto const& triangles = source.getTriangles();
	for (auto it = triangles.cbegin(); it != triangles.cend(); ++it)
	{
		targetTriangle->isValid = it->isValid;
		if (it->isValid)
		{
			targetTriangle->data.normal = pose.linear() * it->data.normal;
		}
		++targetTriangle;
	}
}

}; // namespace DataStructures
}; // namespace SurgSim
