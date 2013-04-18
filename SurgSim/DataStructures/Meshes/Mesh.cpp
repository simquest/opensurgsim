#include "Mesh.h"

using SurgSim::DataStructures::Mesh;
using SurgSim::Math::Vector3d;

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

unsigned int Mesh::createNewVertex(const Vector3d& position)
{
	m_vertexPositions.push_back(position);

	return m_vertexPositions.size() - 1;
}

void Mesh::setVertexPositions(const std::vector<Vector3d>& positions, bool doUpdate)
{
	// TODO: assert that the number of positions matches vertices
	m_vertexPositions = positions;

	update();
}

void Mesh::setVertexPositions(const Mesh &mesh, bool doUpdate)
{
	// TODO: check that meshes have same number of vertices

	m_vertexPositions = mesh.getVertexPositions();

	update();
}

void Mesh::setVertexPositions(const Mesh& mesh1, double percent1, const Mesh& mesh2, double percent2, bool doUpdate)
{
	// TODO: check that meshes have same number of vertices

	for (unsigned int i = 0; i < m_vertexPositions.size(); ++i)
	{
		m_vertexPositions[i] = mesh1.getVertexPosition(i) * percent1 + mesh2.getVertexPosition(i) * percent2;
	}

	update();
}
