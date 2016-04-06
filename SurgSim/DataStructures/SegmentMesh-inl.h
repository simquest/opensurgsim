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

#ifndef SURGSIM_DATASTRUCTURES_SEGMENTMESH_INL_H
#define SURGSIM_DATASTRUCTURES_SEGMENTMESH_INL_H

#include "SurgSim/Framework/Log.h"


namespace SurgSim
{
namespace DataStructures
{

template <class VertexData, class EdgeData>
SegmentMesh<VertexData, EdgeData>::SegmentMesh()
{
}

template <class VertexData, class EdgeData>
SegmentMesh<VertexData, EdgeData>::SegmentMesh(
	const SegmentMesh<VertexData, EdgeData>& other) :
	TriangleMeshType(other)
{
}

template <class VertexData, class EdgeData>
template <class V, class E>
SegmentMesh<VertexData, EdgeData>::SegmentMesh(const SegmentMesh<V, E>& other) :
	TriangleMeshType(other)
{
}

template <class VertexData, class EdgeData>
SegmentMesh<VertexData, EdgeData>::~SegmentMesh()
{
}

template <class VertexData, class EdgeData>
SegmentMesh<VertexData, EdgeData>::SegmentMesh(SegmentMesh<VertexData, EdgeData>&& other) :
	TriangleMeshType(std::move(other))
{
}

template <class VertexData, class EdgeData>
SegmentMesh<VertexData, EdgeData>& SegmentMesh<VertexData, EdgeData>::operator=(
	const SegmentMesh<VertexData, EdgeData>& other)
{
	return TriangleMeshType::operator=(other);
}

template <class VertexData, class EdgeData>
SegmentMesh<VertexData, EdgeData>& SegmentMesh<VertexData, EdgeData>::operator=(
	SegmentMesh<VertexData, EdgeData>&& other)
{
	return TriangleMeshType::operator=(std::move(other));
}

template <class VertexData, class EdgeData>
size_t SegmentMesh<VertexData, EdgeData>::addTriangle(const TriangleType& triangle)
{
	SURGSIM_FAILURE() << "Cannot insert triangle into segment mesh.";
	return static_cast<size_t>(0);
}

template <class VertexData, class EdgeData>
size_t SegmentMesh<VertexData, EdgeData>::getNumTriangles() const
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
	return static_cast<size_t>(0);
}

template <class VertexData, class EdgeData>
const std::vector<typename SegmentMesh<VertexData, EdgeData>::TriangleType>&
SegmentMesh<VertexData, EdgeData>::getTriangles() const
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
	return TriangleMeshType::getTriangles();
}

template <class VertexData, class EdgeData>
std::vector<typename SegmentMesh<VertexData, EdgeData>::TriangleType>&
SegmentMesh<VertexData, EdgeData>::getTriangles()
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
	return TriangleMeshType::getTriangles();
}

template <class VertexData, class EdgeData>
const typename SegmentMesh<VertexData, EdgeData>::TriangleType&
SegmentMesh<VertexData, EdgeData>::getTriangle(size_t id) const
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
	return TriangleMeshType::getTriangle(id);
}

template <class VertexData, class EdgeData>
typename SegmentMesh<VertexData, EdgeData>::TriangleType&
SegmentMesh<VertexData, EdgeData>::getTriangle(size_t id)
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
	return TriangleMeshType::getTriangle(id);
}

template <class VertexData, class EdgeData>
void SegmentMesh<VertexData, EdgeData>::removeTriangle(size_t id)
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
}

template <class VertexData, class EdgeData>
std::array<SurgSim::Math::Vector3d, 3>
SegmentMesh<VertexData, EdgeData>::getTrianglePositions(size_t id) const
{
	using SurgSim::Math::Vector3d;
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
	std::array<Vector3d, 3> result =
	{
		{
			Vector3d::Zero(),
			Vector3d::Zero(),
			Vector3d::Zero()
		}
	};
	return result;
}

template <class VertexData, class EdgeData>
void SegmentMesh<VertexData, EdgeData>::doClearTriangles()
{
	SURGSIM_FAILURE() << "No triangles present in segment mesh.";
}

template <class VertexData, class EdgeData>
void SegmentMesh<VertexData, EdgeData>::doClear()
{
	TriangleMeshType::doClearEdges();
	TriangleMeshType::doClearVertices();
}

template <class VertexData, class EdgeData>
void SurgSim::DataStructures::SegmentMesh<VertexData, EdgeData>::createDefaultEdges()
{
	doClearEdges();
	for (size_t i = 0; i < getNumVertices() - 1; ++i)
	{
		std::array<size_t, 2> vertices = { i, i + 1 };
		EdgeType edge(vertices);
		addEdge(edge);
	}
}


template <class VertexData, class EdgeData>
void SurgSim::DataStructures::SegmentMesh<VertexData, EdgeData>::save(const std::string& fileName, bool asPhysics)
{
	std::fstream out(fileName, std::ios::out);

	if (out.is_open())
	{
		out << "ply" << std::endl;
		out << "format ascii 1.0" << std::endl;
		out << "comment Created by OpenSurgSim, www.opensurgsim.org" << std::endl;
		out << "element vertex " << getNumVertices() << std::endl;
		out << "property float x\nproperty float y\nproperty float z" << std::endl;
		if (asPhysics)
		{
			out << "element 1d_element " << getNumEdges() << std::endl;
			out << "property list uint uint vertex_indices" << std::endl;
			out << "element radius 1" << std::endl;
			out << "property double value" << std::endl;
			out << "element material 1" << std::endl;
			out << "property double mass_density"  << std::endl;
			out << "property double poisson_ratio" << std::endl;
			out << "property double young_modulus" << std::endl;
			out << "element boundary_condition 0" << std::endl;
			out << "property uint vertex_index" << std::endl;
		}
		else
		{
			out << "element edge " << getNumEdges() << std::endl;
			out << "property uint vertex1" << std::endl;
			out << "property uint vertex2" << std::endl;
		}
		out << "end_header" << std::endl;
		for (const auto& vertex : getVertices())
		{
			out << vertex.position[0] << " " << vertex.position[1] << " " << vertex.position[2] << std::endl;
		}

		for (const auto& edge : getEdges())
		{
			if (asPhysics)
			{
				out << "2 ";
			}
			out << edge.verticesId[0] << " " << edge.verticesId[1] << std::endl;
		}
		if (asPhysics)
		{
			out << "0.001" << std::endl;
			out << "900.0 0.45 1.75e9" << std::endl; // Prolene
		}

		if (out.bad())
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
					<< "There was a problem writing " << fileName;
		}

		out.close();
	}
	else
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__
				<< "Could not open " << fileName << " for writing.";
	}
}


}  // namespace DataStructures
}  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_SEGMENTMESH_INL_H
