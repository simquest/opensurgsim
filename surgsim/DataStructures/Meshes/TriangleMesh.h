#ifndef SURGSIM_DATA_STRUCTURES_MESH_H
#define SURGSIM_DATA_STRUCTURES_MESH_H

#include <SurgSim/DataStructures/Meshes/Mesh.h>

namespace SurgSim
{
	namespace DataStructures
	{

		class TriangleMesh : public Mesh
		{
		public:
			struct EdgeTopology
			{
				std::array<unsigned int, 2> vertices;
				std::vector<unsigned int> triangles;
			};

			struct TriangleTopology
			{
				std::array<unsigned int, 3> vertices;
				std::array<unsigned int, 3> edges;
			};

			struct VertexTopology
			{
				std::vector<unsigned int> edges;
				std::vector<unsigned int> triangles;
			};

			/// Default constructor
			TriangleMesh();
			/// Destructor
			~TriangleMesh();

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

			const TriangleTopology& getTriangleTopology(int id) const
			{ 
				return m_triangleTopologies[id];
			}
			const SurgSim::Math::Vector3d& getTriangleNormal(unsigned int id)
			{
				return m_triangleNormals[id];
			}

			void updatePositions(const std::vector<SurgSim::Math::Vector3d>& positions);
			void updatePositionFromOtherMesh(const Mesh &rhs, bool updateNormalAndAabb);
			void updatePositionFromTwoMeshes(const Mesh &m1, double percent1, 
				const Mesh &m2, double percent2, bool updateNormalAndAabb);

			/// Update the mesh topology without initializing the vertices (they will be garbage!)
			void updateTopologyFromOtherMesh(const Mesh& mesh);

			void loadFromSTLFile(const char *filename, const double trans[3], const double orientationQuat[4]);
			void writeToSTLFile(const std::string& filename) const;

			void loadFrom_PlainTriMesh(std::string name, int nbPts, const double *pts, int nbTri, const int *tri_ptID, double scale=1.0);
			void loadFromParticles(std::string name, int nbPts, const double *pts);

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

			// Return -1 if the vertex does not already exist in m_vertices
			// Return the vertex ID if it already exist in m_vertices
			int doesThisVertexAlreadyExist(const double *v) const;

			// Return the new vertex ID
			void registerTriangleInVertex(unsigned int vertexId, unsigned int triangleId);
			void registerEdgeInVertex(unsigned int vertexId, unsigned int edgeId);

			// Return -1 if the edge does not already exist in m_edges
			// Return the edge ID if it already exist in m_edges
			int doesThisEdgeAlreadyExist(int vID0, int vID1) const;
			// Return the new edge ID
			unsigned int createNewEdge(unsigned int vertex0, unsigned int vertex1);
			void registerTriangleInEdge(unsigned int edgeId, unsigned int triangleId);

			// Return the new triangle ID
			unsigned int createNewTriangle(unsigned int vertices[3], unsigned int edges[3]);

			void internalProcessWithVertexAndTriangle();

			bool m_doComputeNormals;
			bool m_doBuildTopology;

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
	}
}

#endif
