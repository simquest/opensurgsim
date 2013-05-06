#include "MeshVertexData.h"

#include <typeinfo>

using SurgSim::DataStructures::MeshVertexData;

MeshVertexData::MeshVertexData()
{
}

MeshVertexData::~MeshVertexData()
{
}

bool MeshVertexData::operator==(const MeshVertexData& data) const
{
	return (typeid(*this) == typeid(data)) && isEqual(data);
}

bool MeshVertexData::operator!=(const MeshVertexData& data) const
{
	return (typeid(*this) != typeid(data)) || ! isEqual(data);
}