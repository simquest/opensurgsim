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

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/TangentSpaceGenerator.h"

#include <osg/TriangleIndexFunctor>
#include <osg/Vec3>
#include <osg/Array>

namespace SurgSim
{
namespace Graphics
{

GenerateTangentSpaceTriangleIndexFunctor::GenerateTangentSpaceTriangleIndexFunctor() :
	m_vertexArray(0),
	m_normalArray(0),
	m_textureCoordArray(0),
	m_tangentArray(0),
	m_bitangentArray(0),
	m_isDeformableArray(0),
	m_createOrthonormalBasis(false)
{
}

void GenerateTangentSpaceTriangleIndexFunctor::setBasisOrthonormality(bool orthonormal)
{
	m_createOrthonormalBasis = orthonormal;
}
bool GenerateTangentSpaceTriangleIndexFunctor::getBasisOrthonormality()
{
	return m_createOrthonormalBasis;
}

void GenerateTangentSpaceTriangleIndexFunctor::set(const osg::Vec3Array* vertexArray, const osg::Vec3Array* normalArray,
		const osg::Vec2Array* textureCoordArray,
		osg::Vec4Array* tangentArray, osg::Vec4Array* bitangentArray,
		const osg::UShortArray* isDeformable)
{
	m_vertexArray = vertexArray;
	m_normalArray = normalArray;
	m_textureCoordArray = textureCoordArray;
	m_tangentArray = tangentArray;
	m_bitangentArray = bitangentArray;
	m_isDeformableArray = isDeformable;

	size_t numVertices = m_vertexArray->size();

	SURGSIM_ASSERT(m_vertexArray != nullptr) << "Need vertex array to generate normals!";
	SURGSIM_ASSERT(m_normalArray != nullptr) << "Need normal array to store normals!";
	SURGSIM_ASSERT(m_normalArray->size() == numVertices)
			<< "Size of normal array must match the number of vertices to generate tangent space!";
	SURGSIM_ASSERT(m_tangentArray != nullptr) << "Need tangent array to store tangent space!";
	SURGSIM_ASSERT(m_tangentArray->size() == numVertices)
			<< "Size of tangent array must match the number of vertices to generate tangent space!";
	SURGSIM_ASSERT(m_bitangentArray != nullptr) <<  "Need bitangent array to store tangent space!";
	SURGSIM_ASSERT(m_bitangentArray->size() == numVertices)
			<< "Size of bitangent array must match the number of vertices to generate tangent space!";
	if (m_isDeformableArray != nullptr)
	{
		SURGSIM_ASSERT(m_isDeformableArray->size() == numVertices)
				<< "Size of vertex deformability array must match the number of vertices!";

	}
}

inline void orthnormalize(const osg::Vec3& normal, osg::Vec4& tangent, osg::Vec4& bitangent,
						  bool createOrthonormalBasis = false)
{
	osg::Vec4 normal4(normal, 0.0);

	// Gram-Schmidt orthogonalize the tangent
	tangent = tangent - normal4 * (normal4 * tangent);
	tangent.normalize();

	if (createOrthonormalBasis)
	{
		osg::Vec3 tangent3 = osg::Vec3(tangent.x(), tangent.y(), tangent.z());
		osg::Vec3 bitangent3 = osg::Vec3(bitangent.x(), bitangent.y(), bitangent.z());

		// Calculate handedness
		float handedness = 1.0f;
		if ((normal ^ tangent3) * bitangent3 < 0.0f)
		{
			handedness = -1.0f;
		}

		bitangent = osg::Vec4((normal ^ tangent3) * handedness, 0.0f);
	}
	else
	{
		// Gram-Schmidt orthogonalize the bitangent
		bitangent = bitangent - normal4 * (normal4 * bitangent);
		bitangent.normalize();
	}
}

void GenerateTangentSpaceTriangleIndexFunctor::orthonormalize()
{
	size_t numVertices = m_vertexArray->size();

	if (m_isDeformableArray)
	{
		for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
		{
			if ((*m_isDeformableArray)[vertexIndex])
			{
				orthnormalize((*m_normalArray)[vertexIndex], (*m_tangentArray)[vertexIndex], (*m_bitangentArray)[vertexIndex],
							  m_createOrthonormalBasis);
			}
		}
	}
	else
	{
		for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
		{
			orthnormalize((*m_normalArray)[vertexIndex], (*m_tangentArray)[vertexIndex], (*m_bitangentArray)[vertexIndex],
						  m_createOrthonormalBasis);
		}
	}
}

void GenerateTangentSpaceTriangleIndexFunctor::reset()
{
	size_t numVertices = m_vertexArray->size();

	if (m_isDeformableArray)
	{
		for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
		{
			if ((*m_isDeformableArray)[vertexIndex])
			{
				(*m_tangentArray)[vertexIndex].set(0.0f, 0.0f, 0.0f, 0.0f);
				(*m_bitangentArray)[vertexIndex].set(0.0f, 0.0f, 0.0f, 0.0f);
			}
		}
	}
	else
	{
		for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
		{
			(*m_tangentArray)[vertexIndex].set(0.0f, 0.0f, 0.0f, 0.0f);
			(*m_bitangentArray)[vertexIndex].set(0.0f, 0.0f, 0.0f, 0.0f);
		}
	}
}

void GenerateTangentSpaceTriangleIndexFunctor::operator()(unsigned int vertexIndex1, unsigned int vertexIndex2,
		unsigned int vertexIndex3)
{
	if (vertexIndex1 == vertexIndex2 || vertexIndex2 == vertexIndex3 || vertexIndex1 == vertexIndex3)
	{
		return;
	}

	const osg::Vec3& v1 = (*m_vertexArray)[vertexIndex1];
	const osg::Vec3& v2 = (*m_vertexArray)[vertexIndex2];
	const osg::Vec3& v3 = (*m_vertexArray)[vertexIndex3];

	const osg::Vec2& w1 = (*m_textureCoordArray)[vertexIndex1];
	const osg::Vec2& w2 = (*m_textureCoordArray)[vertexIndex2];
	const osg::Vec2& w3 = (*m_textureCoordArray)[vertexIndex3];

	float x1 = v2.x() - v1.x();
	float x2 = v3.x() - v1.x();
	float y1 = v2.y() - v1.y();
	float y2 = v3.y() - v1.y();
	float z1 = v2.z() - v1.z();
	float z2 = v3.z() - v1.z();

	float s1 = w2.x() - w1.x();
	float s2 = w3.x() - w1.x();
	float t1 = w2.y() - w1.y();
	float t2 = w3.y() - w1.y();

	float r = 1.0f / (s1 * t2 - s2 * t1);
	osg::Vec4 tangent((t2 * x1 - t1 * x2) * r, (t2 * y1 - t1 * y2) * r, (t2 * z1 - t1 * z2) * r, 0.0f);
	osg::Vec4 bitangent((s1 * x2 - s2 * x1) * r, (s1 * y2 - s2 * y1) * r, (s1 * z2 - s2 * z1) * r, 0.0f);

	if (!m_isDeformableArray || (m_isDeformableArray && (*m_isDeformableArray)[vertexIndex1]))
	{
		(*m_tangentArray)[vertexIndex1] += tangent;
		(*m_bitangentArray)[vertexIndex1] += bitangent;
	}

	if (!m_isDeformableArray || (m_isDeformableArray && (*m_isDeformableArray)[vertexIndex2]))
	{
		(*m_tangentArray)[vertexIndex2] += tangent;
		(*m_bitangentArray)[vertexIndex2] += bitangent;
	}

	if (!m_isDeformableArray || (m_isDeformableArray && (*m_isDeformableArray)[vertexIndex3]))
	{
		(*m_tangentArray)[vertexIndex3] += tangent;
		(*m_bitangentArray)[vertexIndex3] += bitangent;
	}
}

TangentSpaceGenerator::TangentSpaceGenerator(int textureCoordUnit, int tangentAttribIndex, int bitangentAttribIndex,
		int isDeformableAttribIndex) :
	osg::NodeVisitor(),
	m_textureCoordUnit(textureCoordUnit),
	m_tangentAttribIndex(tangentAttribIndex),
	m_bitangentAttribIndex(bitangentAttribIndex),
	m_isDeformableAttribIndex(isDeformableAttribIndex)
{
	setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
}

TangentSpaceGenerator::~TangentSpaceGenerator()
{
}

void TangentSpaceGenerator::setBasisOrthonormality(bool orthonormal)
{
	m_createOrthonormalBasis = orthonormal;
}
bool TangentSpaceGenerator::getBasisOrthonormality()
{
	return m_createOrthonormalBasis;
}

void TangentSpaceGenerator::apply(osg::Geode& geode)
{
	for (unsigned int i = 0; i < geode.getNumDrawables(); i++)
	{
		osg::Geometry* geometry = geode.getDrawable(i)->asGeometry();
		if (geometry)
		{
			generateTangentSpace(geometry, m_textureCoordUnit, m_tangentAttribIndex, m_bitangentAttribIndex,
								 m_createOrthonormalBasis, m_isDeformableAttribIndex);
		}
	}
}

void TangentSpaceGenerator::generateTangentSpace(osg::Geometry* geometry, int textureCoordUnit, int tangentAttribIndex,
		int bitangentAttribIndex,
		bool orthonormal, int isDeformableAttribIndex)
{

	auto logger = SurgSim::Framework::Logger::getLogger("Graphics/TangetSpaceGenerator");

	osg::Vec3Array* vertexArray = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
	if (!vertexArray)
	{
		SURGSIM_LOG_WARNING(logger) << "No Vertices found, could not produce tangents.";
		return;
	}

	osg::Vec3Array* normalArray = dynamic_cast<osg::Vec3Array*>(geometry->getNormalArray());
	if (!normalArray || normalArray->size() != vertexArray->size())
	{
		SURGSIM_LOG_WARNING(logger) << "No Normals found, could not produce tangents.";
		return;
	}

	osg::Vec2Array* textureCoordArray = dynamic_cast<osg::Vec2Array*>(geometry->getTexCoordArray(textureCoordUnit));
	if (!textureCoordArray || textureCoordArray->size() != vertexArray->size())
	{
		SURGSIM_LOG_WARNING(logger) << "No Texture Coordinates found, could not produce tangents.";
		return;
	}

	bool needToGenerateAllTangents = false;

	osg::Vec4Array* tangentArray = dynamic_cast<osg::Vec4Array*>(geometry->getVertexAttribArray(tangentAttribIndex));
	if (!tangentArray || (tangentArray && tangentArray->size() != vertexArray->size()))
	{
		tangentArray = new osg::Vec4Array(vertexArray->size());
		geometry->setVertexAttribArray(tangentAttribIndex, tangentArray);
		geometry->setVertexAttribBinding(tangentAttribIndex, osg::Geometry::BIND_PER_VERTEX);
		needToGenerateAllTangents = true;
	}

	osg::Vec4Array* bitangentArray = dynamic_cast<osg::Vec4Array*>(geometry->getVertexAttribArray(bitangentAttribIndex));
	if (!bitangentArray || (bitangentArray && bitangentArray->size() != vertexArray->size()))
	{
		bitangentArray = new osg::Vec4Array(vertexArray->size());
		geometry->setVertexAttribArray(bitangentAttribIndex, bitangentArray);
		geometry->setVertexAttribBinding(bitangentAttribIndex, osg::Geometry::BIND_PER_VERTEX);
		needToGenerateAllTangents = true;
	}

	const osg::UShortArray* isDeformableArray = 0;
	if (isDeformableAttribIndex >= 0 && !needToGenerateAllTangents)
	{
		isDeformableArray = static_cast<osg::UShortArray*>(geometry->getVertexAttribArray(isDeformableAttribIndex));
	}

	osg::TriangleIndexFunctor<GenerateTangentSpaceTriangleIndexFunctor> tangentSpaceGenerator;
	tangentSpaceGenerator.setBasisOrthonormality(orthonormal);
	tangentSpaceGenerator.set(vertexArray, normalArray, textureCoordArray, tangentArray, bitangentArray, isDeformableArray);
	tangentSpaceGenerator.reset();
	geometry->accept(tangentSpaceGenerator);
	tangentSpaceGenerator.orthonormalize();
}


}
}
