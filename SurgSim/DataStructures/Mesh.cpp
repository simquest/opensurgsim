#include "Mesh.h"

#include <SurgSim/Framework/Assert.h>

using SurgSim::DataStructures::Mesh;
using SurgSim::Math::Vector3d;

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::setVertexPositions(const std::vector<Vector3d>& positions, bool doUpdate)
{
	SURGSIM_ASSERT(m_vertices.size() == positions.size()) << "Number of positions must match number of vertices.";

	for (unsigned int i = 0; i < m_vertices.size(); ++i)
	{
		m_vertices[i].position = positions[i];
	}

	if (doUpdate)
	{
		update();
	}
}

bool Mesh::operator==(const Mesh& mesh) const
{
	return (typeid(*this) == typeid(mesh)) && isEqual(mesh);
}

bool Mesh::operator!=(const Mesh& mesh) const
{
	return (typeid(*this) != typeid(mesh)) || ! isEqual(mesh);
}