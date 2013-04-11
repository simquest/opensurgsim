#ifndef SURGSIM_DATA_STRUCTURES_MESH_H
#define SURGSIM_DATA_STRUCTURES_MESH_H

#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>

namespace SurgSim
{
	namespace DataStructures
	{

		class Edge;
		class Triangle;
		class Vertex;

		class Mesh
		{
		public:
			/// Default constructor
			Mesh();
			/// Destructor
			~Mesh();

			/// Reset to empty
			void reset();

			unsigned int numVertices() const
			{ 
				m_vertices.size();
			}
			unsigned int numEdges() const
			{
				m_edges.size();
			}
			unsigned int numTriangles() const
			{
				m_triangles.size();
			}

			const Vertex& getVertex(int id) const
			{
				return m_vertices[id];
			}
			const Edge& getEdge(int id) const
			{
				return m_edges[id];
			}
			const Triangle& getTriangle(int id) const
			{ 
				return m_triangles[id];
			}

			void update()
			{
				doUpdate();
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

			bool writeMeshToFile(char *filename);
			bool readMeshFromFile(char *filename);

		private:

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
			unsigned int createNewVertex(const SurgSim::Math::Vector3d& position);
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

			std::vector< std::shared_ptr<Vertex> >   m_vertices;
			std::vector< std::shared_ptr<Edge> >     m_edges;
			std::vector< std::shared_ptr<Triangle> > m_triangles;
		};
	}
}

#endif
