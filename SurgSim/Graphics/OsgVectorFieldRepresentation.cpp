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

#include <SurgSim/Graphics/OsgVectorFieldRepresentation.h>

namespace SurgSim
{
namespace Graphics
{

template <>
void OsgVectorFieldRepresentation<SurgSim::Math::Vector4d>::doUpdate(double dt)
{
	if (m_vertices != nullptr)
	{
		// std::vector< Vertex<Data> > vertices
		auto vertices = m_vertices->getVertices();
		size_t count = vertices.size();

		if (count != static_cast<size_t>(m_drawArrays->getCount()) &&
			count > m_vertexData->size())
		{
			m_vertexData->resize(count*2);
		}

		osg::ref_ptr<osg::Vec4Array> osgColors = new osg::Vec4Array;
		// Copy OSS Vertices (data structure) into osg vertices
		for (size_t i = 0; i < count; ++i)
		{
			float x = static_cast<float>(vertices[i].position[0]);
			float y = static_cast<float>(vertices[i].position[1]);
			float z = static_cast<float>(vertices[i].position[2]);

			float newX = static_cast<float>(vertices[i].data.position[0]);
			float newY = static_cast<float>(vertices[i].data.position[1]);
			float newZ = static_cast<float>(vertices[i].data.position[2]);

			(*m_vertexData)[2*i][0] = x;
			(*m_vertexData)[2*i][1] = y;
			(*m_vertexData)[2*i][2] = z;
			(*m_vertexData)[2*i+1][0] = x + newX/10;
			(*m_vertexData)[2*i+1][1] = y + newY/10;
			(*m_vertexData)[2*i+1][2] = z + newZ/10;

			auto color = SurgSim::Graphics::toOsg(vertices[i].data.data); 
			osgColors->push_back(color);
			osgColors->push_back(color);
		}
		m_lineGeometry->setColorArray(osgColors, osg::Array::Binding::BIND_PER_VERTEX);
		m_drawArrays->setCount(count*2);
		m_drawArrays->dirty();
		m_lineGeometry->dirtyBound();
		m_lineGeometry->dirtyDisplayList();
	}
	else
	{
		if (m_drawArrays->getCount() != 0)
		{
			m_drawArrays->setCount(0);
			m_drawArrays->dirty();
			m_lineGeometry->dirtyBound();
		}
	}

}

}; // Graphics
}; // SurgSim
