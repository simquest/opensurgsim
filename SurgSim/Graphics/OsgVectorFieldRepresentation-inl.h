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

#ifndef SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_INL
#define SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_INL

#include <osg/LineWidth>
#include <osg/PositionAttitudeTransform>
#include <osg/StateAttribute>
#include <osg/Geometry>

#include <SurgSim/Graphics/OsgConversions.h>

namespace SurgSim
{
namespace Graphics
{

template <class Data>
OsgVectorFieldRepresentation<Data>::OsgVectorFieldRepresentation(const std::string& name) :
	Representation(name),
	VectorFieldRepresentation<Data>(name),
	OsgRepresentation(name),
	m_vertices(),
	m_colors()
{
	m_vertexData = new osg::Vec3Array;
	m_geometry = new osg::Geometry();
	m_geometry->setVertexArray(m_vertexData);

	// At this stage there are no vertices in there
	m_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, m_vertexData->size());
	
	m_geometry->addPrimitiveSet(m_drawArrays);
	m_geometry->setUseDisplayList(false);
	m_geometry->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	m_line = new osg::LineWidth(1.0f);
	m_geometry->getOrCreateStateSet()->setAttribute(m_line, osg::StateAttribute::ON);

	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(m_geometry);
	m_transform->addChild(geode);
}


template <class Data>
OsgVectorFieldRepresentation<Data>::~OsgVectorFieldRepresentation()
{
}

template <class Data>
void OsgVectorFieldRepresentation<Data>::doUpdate(double dt)
{
	if (m_vertices != nullptr)
	{
		// std::vector< Vertex<Data> > vertices
		auto vertices = m_vertices->getVertices();
		size_t count = vertices.size();

		if (count != static_cast<size_t>(m_drawArrays->getCount()) &&
			count > m_vertexData->size())
		{
			m_vertexData->resize(count);
		}
		
		// Copy OSS Vertices (data structure) into osg vertices
		for (size_t i = 0; i < count; ++i)
		{
			(*m_vertexData)[i][0] = static_cast<float>(vertices[i].position[0]);
			(*m_vertexData)[i][1] = static_cast<float>(vertices[i].position[1]);
			(*m_vertexData)[i][2] = static_cast<float>(vertices[i].position[2]);
		}

		m_drawArrays->setCount(count);
		m_drawArrays->dirty();
		m_geometry->dirtyBound();
		m_geometry->dirtyDisplayList();
	}
	else
	{
		if (m_drawArrays->getCount() != 0)
		{
			m_drawArrays->setCount(0);
			m_drawArrays->dirty();
			m_geometry->dirtyBound();
		}
	}
}


template <class Data>
void OsgVectorFieldRepresentation<Data>::setVertices(std::shared_ptr<SurgSim::DataStructures::Vertices<Data>> mesh)
{
	m_vertices = mesh;
}


template <class Data>
std::shared_ptr<SurgSim::DataStructures::Vertices<Data>> OsgVectorFieldRepresentation<Data>::getVertices() const
{
	return m_vertices;
}


template <class Data>
void OsgVectorFieldRepresentation<Data>::setLineWidth(double width)
{
	m_line->setWidth(width);
}

template <class Data>
double OsgVectorFieldRepresentation<Data>::getLineWidth() const
{
	return static_cast<double>(m_line->getWidth());
}

template <class Data>
void OsgVectorFieldRepresentation<Data>::setColors(const std::vector<SurgSim::Math::Vector4d>& colors)
{
	// Set the color of the particles to one single color by default
	osg::Vec4Array* osgColors = new osg::Vec4Array;
	for (auto it = std::begin(colors); it != std::end(colors); ++it)
	{
		osgColors->push_back(SurgSim::Graphics::toOsg(*it));
	}
	m_geometry->setColorArray(osgColors);
	m_geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	m_colors = colors;
}

template <class Data>
std::vector<SurgSim::Math::Vector4d> OsgVectorFieldRepresentation<Data>::getColors() const
{
	return m_colors;
}

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_INL