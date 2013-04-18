#ifndef SURGSIM_DATA_STRUCTURES_TRIANGLE_MESH_H
#define SURGSIM_DATA_STRUCTURES_TRIANGLE_MESH_H

#include <SurgSim/DataStructures/Meshes/Mesh.h>

#include <unordered_set>

namespace SurgSim
{

namespace DataStructures
{

class TriangleMesh : public Mesh
{
public:
	struct VertexTopology
	{
		std::unordered_set<unsigned int> edges;
		std::unordered_set<unsigned int> triangles;
	};

	struct EdgeTopology
	{
		EdgeTopology(unsigned int vertex0, unsigned int vertex1)
		{
			vertices[0] = vertex0;
			vertices[1] = vertex1;
		}
		std::array<unsigned int, 2> vertices;
		std::unordered_set<unsigned int> triangles;
	};

	struct TriangleTopology
	{
		TriangleTopology(const unsigned int vertices[3], const unsigned int edges[3])
		{
			this->vertices[0] = vertices[0];
			this->vertices[1] = vertices[1];
			this->vertices[2] = vertices[2];
			this->edges[0] = edges[0];
			this->edges[1] = edges[1];
			this->edges[2] = edges[2];
		}
		TriangleTopology(unsigned int vertex0, unsigned int vertex1, unsigned int vertex2,
			unsigned int edge0, unsigned int edge1, unsigned int edge2)
		{
			vertices[0] = vertex0;
			vertices[1] = vertex1;
			vertices[2] = vertex2;
			edges[0] = edge0;
			edges[1] = edge1;
			edges[2] = edge2;
		}
		std::array<unsigned int, 3> vertices;
		std::array<unsigned int, 3> edges;
	};

	/// Default constructor
	TriangleMesh();
	/// Destructor
	~TriangleMesh();

	// Return the new edge ID
	unsigned int createNewEdge(unsigned int vertex0, unsigned int vertex1);
	// Return the new triangle ID
	unsigned int createNewTriangle(unsigned int vertices[3], unsigned int edges[3]);

	unsigned int numEdges() const
	{
		m_edgeTopologies.size();
	}
	unsigned int numTriangles() const
	{
		m_triangleTopologies.size();
	}

	const EdgeTopology& getEdgeTopology(unsigned int id) const
	{
		return m_edgeTopologies[id];
	}
	const SurgSim::Math::Vector3d& getEdgeNormal(unsigned int id)
	{
		return m_edgeNormals[id];
	}

	const TriangleTopology& getTriangleTopology(unsigned int id) const
	{ 
		return m_triangleTopologies[id];
	}
	const SurgSim::Math::Vector3d& getTriangleNormal(unsigned int id)
	{
		return m_triangleNormals[id];
	}

protected:
	virtual void doResetEdges()
	{
		m_edgeTopologies.clear();
		m_edgeNormals.clear();
	}
	virtual void doResetTriangles()
	{
		m_triangleTopologies.clear();
		m_triangleNormals.clear();
	}
	virtual void doResetVertices()
	{
		Mesh::doResetVertices();
		m_vertexNormals.clear();
	}
private:

	virtual void doReset()
	{
		doResetTriangles();
		doResetEdges();
		doResetVertices();
	}

	virtual void doUpdate()
	{
		doUpdateTriangles();
		doUpdateEdges();
		doUpdateVertices();
	}

	virtual void doUpdateTriangles();
	virtual void doUpdateEdges();
	virtual void doUpdateVertices();

	virtual void computeNormals()
	{
		computeTriangleNormals();
		computeEdgeNormals();
		computeVertexNormals();
	}

	virtual void computeTriangleNormals();
	virtual void computeEdgeNormals();
	virtual void computeVertexNormals();

	// Return the new vertex ID
	void registerTriangleInVertex(unsigned int vertexId, unsigned int triangleId);
	void registerEdgeInVertex(unsigned int vertexId, unsigned int edgeId);
	
	void registerTriangleInEdge(unsigned int edgeId, unsigned int triangleId);

	bool m_doComputeNormals;
	bool m_doBuildExtendedTopology;

	/// Topology of each vertex in the mesh.
	std::vector<VertexTopology> m_vertexTopologies;
	/// Normal of each vertex in the mesh.
	std::vector<SurgSim::Math::Vector3d> m_vertexNormals;

	/// Topology of each edge in the mesh.
	std::vector<EdgeTopology> m_edgeTopologies;
	/// Normal of each edge in the mesh.
	std::vector<SurgSim::Math::Vector3d> m_edgeNormals;

	/// Topology of each triangle in the mesh.
	std::vector<TriangleTopology> m_triangleTopologies;
	/// Normal of each edge in the mesh.
	std::vector<SurgSim::Math::Vector3d> m_triangleNormals;
};

};  // namespace DataStructures

};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_TRIANGLE_MESH_H
