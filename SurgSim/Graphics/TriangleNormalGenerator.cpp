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

#include "SurgSim/Graphics/TriangleNormalGenerator.h"
#include "SurgSim/Framework/Log.h"

#include <osg/TriangleIndexFunctor>
#include <osg/Vec3>
#include <osg/Array>


namespace SurgSim
{
namespace Graphics
{

TriangleNormalGenerator::TriangleNormalGenerator() :
	m_vertexArray(nullptr),
	m_normalArray(nullptr),
	m_size(0)
{
}

void TriangleNormalGenerator::set(
	osg::Vec3Array* vertexArray,
	osg::Vec3Array* normalArray)
{

	SURGSIM_ASSERT(vertexArray != nullptr) << "Need vertex array to generate normals!";
	SURGSIM_ASSERT(normalArray != nullptr) << "Need normal array to store normals!";
	SURGSIM_ASSERT(normalArray->size() == vertexArray->size()) << "Vertex and normal array need to have same size";

	m_vertexArray = vertexArray;
	m_normalArray = normalArray;
	m_size = vertexArray->size();
}

void TriangleNormalGenerator::normalize()
{
	for(osg::Vec3Array::iterator normalIt = m_normalArray->begin(); normalIt != m_normalArray->end(); ++normalIt)
	{
		(*normalIt).normalize();
	}
}

void TriangleNormalGenerator::reset()
{
	for(osg::Vec3Array::iterator normalIt = m_normalArray->begin(); normalIt != m_normalArray->end(); ++normalIt)
	{
		(*normalIt).set(0.0f, 0.0f, 0.0f);
	}

}

void TriangleNormalGenerator::operator() (size_t vertexIndex1,
										  size_t vertexIndex2,
										  size_t vertexIndex3)
{
	if (vertexIndex1 == vertexIndex2 || vertexIndex2 == vertexIndex3 || vertexIndex1 == vertexIndex3)
	{
		return;
	}

	const osg::Vec3& v1 = (*m_vertexArray)[vertexIndex1];
	const osg::Vec3& v2 = (*m_vertexArray)[vertexIndex2];
	const osg::Vec3& v3 = (*m_vertexArray)[vertexIndex3];
	osg::Vec3 normal = (v2-v1) ^ (v3-v1);
	normal.normalize();

	(*m_normalArray)[vertexIndex1] += normal;
	(*m_normalArray)[vertexIndex2] += normal;
	(*m_normalArray)[vertexIndex3] += normal;
}

osg::TriangleIndexFunctor<TriangleNormalGenerator> createNormalGenerator(
	osg::Vec3Array* vertexArray,
	osg::Vec3Array* normalArray)
{
	osg::TriangleIndexFunctor<TriangleNormalGenerator> result;
	result.set(vertexArray, normalArray);
	return result;
}

}; // Graphics
}; // SurgSim
