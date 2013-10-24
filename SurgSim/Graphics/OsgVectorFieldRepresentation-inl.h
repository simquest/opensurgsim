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

#ifndef SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_INL_H
#define SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_INL_H

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/ref_ptr>
#include <osg/StateAttribute>

#include <SurgSim/DataStructures/Vertex.h>
#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/Graphics/OsgConversions.h>

namespace SurgSim
{
namespace Graphics
{

using SurgSim::DataStructures::Vertex;
using SurgSim::DataStructures::Vertices;

template <class Data>
OsgVectorFieldRepresentation<Data>::OsgVectorFieldRepresentation(const std::string& name) :
	Representation(name),
	VectorFieldRepresentation<Data>(name),
	OsgRepresentation(name),
	m_vertices(nullptr)
{
	m_vertexData = new osg::Vec3Array;
	m_lineGeometry = new osg::Geometry;
	m_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINES);
	m_line = new osg::LineWidth;

	m_lineGeometry->setVertexArray(m_vertexData);
	m_lineGeometry->addPrimitiveSet(m_drawArrays);
	m_lineGeometry->setUseDisplayList(false);
	m_lineGeometry->setDataVariance(osg::Object::DYNAMIC);
	m_lineGeometry->getOrCreateStateSet()->setAttribute(m_line, osg::StateAttribute::ON);

	// Put a point at origin of the coordinate
	osg::ref_ptr<osg::Geometry> pointGeometry = new osg::Geometry;
	osg::ref_ptr<osg::Point> point = new osg::Point(2.0f);
	osg::ref_ptr<osg::DrawElementsUInt> pointElement = new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS);
	pointElement->push_back(0);
	osg::ref_ptr<osg::Vec3Array> pointData = new osg::Vec3Array;
	pointData->push_back(osg::Vec3(0.0, 0.0, 0.0));

	pointGeometry->setVertexArray(pointData);
	pointGeometry->addPrimitiveSet(pointElement);
	pointGeometry->setUseDisplayList(false);
	pointGeometry->getOrCreateStateSet()->setAttribute(point, osg::StateAttribute::ON);

	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(m_lineGeometry);
	geode->addDrawable(pointGeometry);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
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
			m_vertexData->resize(count*2);
		}

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
		}
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


template <class Data>
void OsgVectorFieldRepresentation<Data>::setVertices(std::shared_ptr< Vertices< Vertex<Data> > > mesh)
{
	m_vertices = mesh;
}


template <class Data>
std::shared_ptr< Vertices< Vertex<Data> > > OsgVectorFieldRepresentation<Data>::getVertices() const
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


template <>
void OsgVectorFieldRepresentation<SurgSim::Math::Vector4d>::doUpdate(double dt);

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_INL_H