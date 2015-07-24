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

namespace
{

void orthogonalize(const osg::Vec3& normal, osg::Vec4* tangent, osg::Vec4* bitangent,
				   bool createOrthonormalBasis = false)
{
	SURGSIM_ASSERT(tangent != nullptr) << "Tanget parameter can't be nullptr.";
	SURGSIM_ASSERT(bitangent != nullptr) << "BiTangent parameter can't be nullptr.";

	osg::Vec4 normal4(normal, 0.0);
	// Gram-Schmidt orthogonalize the tangent
	(*tangent) = (*tangent) - normal4 * (normal4 * (*tangent));
	tangent->normalize();

	if (createOrthonormalBasis)
	{
		osg::Vec3 tangent3 = osg::Vec3(tangent->x(), tangent->y(), tangent->z());
		osg::Vec3 bitangent3 = osg::Vec3(bitangent->x(), bitangent->y(), bitangent->z());
		osg::Vec3 cross = normal ^ tangent3;

		// Calculate handedness
		float handedness = 1.0f;
		if ((cross * bitangent3) < 0.0f)
		{
			handedness = -1.0f;
		}

		(*bitangent) = osg::Vec4(cross * handedness, 0.0f);
	}
	else
	{
		// Gram-Schmidt orthogonalize the bitangent
		(*bitangent) = (*bitangent) - normal4 * (normal4 * (*bitangent));
		bitangent->normalize();
	}
}

}

namespace SurgSim
{
namespace Graphics
{

GenerateTangentSpaceTriangleIndexFunctor::GenerateTangentSpaceTriangleIndexFunctor() :
	m_vertexArray(nullptr),
	m_normalArray(nullptr),
	m_textureCoordArray(nullptr),
	m_tangentArray(nullptr),
	m_bitangentArray(nullptr),
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

void GenerateTangentSpaceTriangleIndexFunctor::set(
	const osg::Vec3Array* vertexArray,
	const osg::Vec3Array* normalArray,
	const osg::Vec2Array* textureCoordArray,
	osg::Vec4Array* tangentArray,
	osg::Vec4Array* bitangentArray)
{

	SURGSIM_ASSERT(vertexArray != nullptr) << "Need vertex array to generate normals!";
	size_t numVertices = vertexArray->size();

	SURGSIM_ASSERT(normalArray != nullptr) << "Need normal array to store normals!";
	SURGSIM_ASSERT(normalArray->size() == numVertices)
			<< "Size of normal array must match the number of vertices to generate tangent space!";

	SURGSIM_ASSERT(tangentArray != nullptr) << "Need tangent array to store tangent space!";
	SURGSIM_ASSERT(tangentArray->size() == numVertices)
			<< "Size of tangent array must match the number of vertices to generate tangent space!";

	SURGSIM_ASSERT(bitangentArray != nullptr) <<  "Need bitangent array to store tangent space!";
	SURGSIM_ASSERT(bitangentArray->size() == numVertices)
			<< "Size of bitangent array must match the number of vertices to generate tangent space!";

	m_vertexArray = vertexArray;
	m_normalArray = normalArray;
	m_textureCoordArray = textureCoordArray;
	m_tangentArray = tangentArray;
	m_bitangentArray = bitangentArray;
}

void GenerateTangentSpaceTriangleIndexFunctor::orthogonalize()
{
	size_t numVertices = m_vertexArray->size();

	for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
	{
		::orthogonalize((*m_normalArray)[vertexIndex],
						 &(*m_tangentArray)[vertexIndex],
						 &(*m_bitangentArray)[vertexIndex],
						 m_createOrthonormalBasis);
	}
}

void GenerateTangentSpaceTriangleIndexFunctor::reset()
{
	size_t numVertices = m_vertexArray->size();

	for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
	{
		(*m_tangentArray)[vertexIndex].set(0.0f, 0.0f, 0.0f, 0.0f);
		(*m_bitangentArray)[vertexIndex].set(0.0f, 0.0f, 0.0f, 0.0f);
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

	(*m_tangentArray)[vertexIndex1] += tangent;
	(*m_bitangentArray)[vertexIndex1] += bitangent;

	(*m_tangentArray)[vertexIndex2] += tangent;
	(*m_bitangentArray)[vertexIndex2] += bitangent;

	(*m_tangentArray)[vertexIndex3] += tangent;
	(*m_bitangentArray)[vertexIndex3] += bitangent;
}

TangentSpaceGenerator::TangentSpaceGenerator(int textureCoordUnit, int tangentAttribIndex, int bitangentAttribIndex) :
	osg::NodeVisitor(),
	m_textureCoordUnit(textureCoordUnit),
	m_tangentAttribIndex(tangentAttribIndex),
	m_bitangentAttribIndex(bitangentAttribIndex)
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
								 m_createOrthonormalBasis);
		}
	}
}

void TangentSpaceGenerator::generateTangentSpace(osg::Geometry* geometry, int textureCoordUnit, int tangentAttribIndex,
		int bitangentAttribIndex,
		bool orthonormal)
{

	auto logger = SurgSim::Framework::Logger::getLogger("Graphics/TangetSpaceGenerator");

	osg::Vec3Array* vertexArray = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());
	if (vertexArray == nullptr)
	{
		SURGSIM_LOG_WARNING(logger) << "No Vertices found, could not produce tangents.";
		return;
	}

	osg::Vec3Array* normalArray = dynamic_cast<osg::Vec3Array*>(geometry->getNormalArray());
	if (normalArray == nullptr || normalArray->size() != vertexArray->size())
	{
		SURGSIM_LOG_WARNING(logger) << "No Normals found, or mismatch in array sizes, could not produce tangents.";
		return;
	}

	osg::Vec2Array* textureCoordArray =
		dynamic_cast<osg::Vec2Array*>(geometry->getTexCoordArray(textureCoordUnit));
	if (textureCoordArray == nullptr || textureCoordArray->size() != vertexArray->size())
	{
		SURGSIM_LOG_WARNING(logger)
				<< "No Texture Coordinates found, or mismatch in array sizes could not produce tangents.";
		return;
	}

	bool didUpdateGeom = false;

	osg::Vec4Array* tangentArray =
		dynamic_cast<osg::Vec4Array*>(geometry->getVertexAttribArray(tangentAttribIndex));
	if (tangentArray == nullptr || tangentArray->size() != vertexArray->size())
	{
		tangentArray = new osg::Vec4Array(vertexArray->size());
		geometry->setVertexAttribArray(tangentAttribIndex, tangentArray);
		geometry->setVertexAttribBinding(tangentAttribIndex, osg::Geometry::BIND_PER_VERTEX);
		didUpdateGeom = true;
	}

	osg::Vec4Array* bitangentArray =
		dynamic_cast<osg::Vec4Array*>(geometry->getVertexAttribArray(bitangentAttribIndex));
	if (bitangentArray == nullptr || bitangentArray->size() != vertexArray->size())
	{
		bitangentArray = new osg::Vec4Array(vertexArray->size());
		geometry->setVertexAttribArray(bitangentAttribIndex, bitangentArray);
		geometry->setVertexAttribBinding(bitangentAttribIndex, osg::Geometry::BIND_PER_VERTEX);
		didUpdateGeom = true;
	}

	osg::TriangleIndexFunctor<GenerateTangentSpaceTriangleIndexFunctor> tangentSpaceGenerator;
	tangentSpaceGenerator.setBasisOrthonormality(orthonormal);
	tangentSpaceGenerator.set(vertexArray, normalArray, textureCoordArray, tangentArray, bitangentArray);
	tangentSpaceGenerator.reset();
	geometry->accept(tangentSpaceGenerator);
	tangentSpaceGenerator.orthogonalize();

	if (didUpdateGeom)
	{
		// geometry->dirtyDisplayList();
		geometry->setUseDisplayList(false);
		geometry->dirtyBound();
		geometry->setDataVariance(osg::Object::DYNAMIC);
	}
}


}
}
