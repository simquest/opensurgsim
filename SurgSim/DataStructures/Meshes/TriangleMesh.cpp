#include "TriangleMesh.h"

#include <math.h>

using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::Vector3d;

TriangleMesh::TriangleMesh()
{
}

TriangleMesh::~TriangleMesh()
{
}

static inline bool sameEdge(unsigned int edge1vertex1, unsigned int edge1vertex2, 
	unsigned int edge2vertex1, unsigned int edge2vertex2)
{ 
	return ( edge1vertex1 == edge2vertex1 && edge1vertex2 == edge2vertex2 ) || 
		( edge1vertex1 == edge2vertex2 && edge1vertex2 == edge2vertex1 );
}

static inline bool sharesEdge(const TriangleMesh::TriangleTopology& triangle1, 
	const TriangleMesh::TriangleTopology& triangle2)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (triangle1.edges[i] == triangle2.edges[j])
			{
				return true;
			}
		}
	}

	return false;
}

unsigned int TriangleMesh::createNewEdge(unsigned int vertex0, unsigned int vertex1)
{
	EdgeTopology edge(vertex0, vertex1);
	m_edgeTopologies.push_back(edge);

	return m_edgeTopologies.size() - 1;
}

unsigned int TriangleMesh::createNewTriangle(unsigned int vertices[3], unsigned int edges[3])
{
	TriangleTopology triangle(vertices, edges);

	m_triangleTopologies.push_back(triangle);

	unsigned int triangleId = m_triangleTopologies.size() - 1;

	for (int i = 0; i < 3; ++i)
	{
		registerTriangleInVertex(vertices[i] , triangleId);
		registerTriangleInEdge(edges[i], triangleId);
	}

	registerEdgeInVertex(vertices[0], edges[0]); 
	registerEdgeInVertex(vertices[0], edges[1]);

	registerEdgeInVertex(vertices[1], edges[0]);
	registerEdgeInVertex(vertices[1], edges[2]);

	registerEdgeInVertex(vertices[2], edges[1]);
	registerEdgeInVertex(vertices[2], edges[2]);

	return triangleId;
}

void TriangleMesh::registerEdgeInVertex(unsigned int edgeId, unsigned int vertexId)
{
	VertexTopology& vertex = m_vertexTopologies[vertexId];
	vertex.edges.insert(edgeId);
}

void TriangleMesh::registerTriangleInVertex(unsigned int triangleId, unsigned int vertexId)
{
	VertexTopology& vertex = m_vertexTopologies[vertexId];
	vertex.triangles.insert(triangleId);
}

void TriangleMesh::registerTriangleInEdge(unsigned int triangleId, unsigned int edgeId)
{
	EdgeTopology& edge = m_edgeTopologies[edgeId];
	edge.triangles.insert(edgeId);
}

void TriangleMesh::doUpdateTriangles()
{
	if (m_doBuildExtendedTopology && m_doComputeNormals)
	{
		computeTriangleNormals();
	}
}

void TriangleMesh::doUpdateEdges()
{
	if (m_doBuildExtendedTopology && m_doComputeNormals)
	{
		computeEdgeNormals();
	}
}

void TriangleMesh::doUpdateVertices()
{
	if (m_doBuildExtendedTopology && m_doComputeNormals)
	{
		computeVertexNormals();
	}
}

void TriangleMesh::computeTriangleNormals()
{
	m_triangleNormals.clear();
	m_triangleNormals.reserve(m_triangleTopologies.size());

	for (auto it = m_triangleTopologies.cbegin(); it != m_triangleTopologies.cend(); ++it)
	{
		Vector3d u = getVertexPosition(it->vertices[1]) - getVertexPosition(it->vertices[0]);
		Vector3d v = getVertexPosition(it->vertices[2]) - getVertexPosition(it->vertices[0]);

		Vector3d normal = u.cross(v);
		normal.normalize();
		m_triangleNormals.push_back(normal);
	}
}

void TriangleMesh::computeEdgeNormals()
{
	m_edgeNormals.clear();
	m_edgeNormals.reserve(m_edgeTopologies.size());

	for (auto edgeIt = m_edgeTopologies.cbegin(); edgeIt != m_edgeTopologies.cend(); ++edgeIt)
	{
		Vector3d normal(0.0, 0.0, 0.0);

		for (auto triangleIdIt = edgeIt->triangles.cbegin(); triangleIdIt != edgeIt->triangles.cend(); ++triangleIdIt)
		{
			normal += getTriangleNormal(*triangleIdIt);
		}

		normal.normalize();
		m_edgeNormals.push_back(normal);
	}
}

void TriangleMesh::computeVertexNormals()
{
	m_vertexNormals.clear();
	m_vertexNormals.reserve(m_vertexTopologies.size());

	for (auto vertexIt = m_vertexTopologies.cbegin(); vertexIt != m_vertexTopologies.cend(); ++vertexIt)
	{
		Vector3d normal(0.0, 0.0, 0.0);

		// Do not count multiple triangles from the same face
		// Example: a rectangle decomposed to 2 triangles cause the same normal to be added twice for the same vertex.
		for (auto triangleIdIt = vertexIt->triangles.cbegin(); triangleIdIt != vertexIt->triangles.cend(); ++triangleIdIt)
		{
			const Vector3d& triangleNormal = getTriangleNormal(*triangleIdIt);

			// Check if the current triangle belongs to the same face as a triangle already added before.
			// If so, it is not valid, and its normal will not contribute to the vertex's normal.
			bool isTriangleValid = true;
			for (auto testIdIt = vertexIt->triangles.cbegin(); testIdIt != triangleIdIt; ++testIdIt)
			{
				const Vector3d& testNormal = getTriangleNormal(*testIdIt);

				// Same normal and the triangles share an edge => multiple triangles for a single face.
				// TODO: Do not hard-code the epsilon.
				if (triangleNormal.dot(testNormal) > 1.0 - 1e-4 &&
					sharesEdge(getTriangleTopology(*triangleIdIt), getTriangleTopology(*testIdIt)))
				{
					isTriangleValid = false;
				}
			}

			// If the triangle is valid, we add its normal component to the vertex normal.
			if (isTriangleValid)
			{
				normal += triangleNormal;
			}
		}

		normal.normalize();
		m_vertexNormals.push_back(normal);
	}
	
}
