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

#include <osg/Geode>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/ref_ptr>
#include <osg/StateAttribute>

#include <SurgSim/Graphics/OsgConversions.h>

namespace SurgSim
{
namespace Graphics
{

OsgVectorFieldRepresentation::OsgVectorFieldRepresentation(const std::string& name) :
	Representation(name),
	VectorFieldRepresentation(name),
	OsgRepresentation(name),
	m_vertices(nullptr),
	m_scale(10.0)
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
	pointData->push_back(osg::Vec3(0.0f, 0.0f, 0.0));

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


OsgVectorFieldRepresentation::~OsgVectorFieldRepresentation()
{
}

void OsgVectorFieldRepresentation::doUpdate(double dt)
{
	if (m_vertices != nullptr)
	{
		// auto = std::vector< Vertex<VectorFieldData> >
		auto vertices = m_vertices->getVertices();
		size_t count = vertices.size();

		if (count != static_cast<size_t>(m_drawArrays->getCount()) &&
			count > m_vertexData->size())
		{
			m_vertexData->resize(count * 2);
		}

		// Assign color to osg line
		osg::ref_ptr<osg::Vec4Array> osgColors;
		if (vertices[0].data.vectorColor.hasValue())
		{
			osgColors = new osg::Vec4Array(count * 2);
			osgColors->setDataVariance(osg::Object::DYNAMIC);
			m_lineGeometry->setColorArray(osgColors, osg::Array::BIND_PER_VERTEX);
			for (size_t i = 0; i < count; ++i)
			{
				// Assign color to the osg line
				osg::Vec4d color = SurgSim::Graphics::toOsg(vertices[i].data.vectorColor.getValue());
				(*osgColors)[2 * i] = color;
				(*osgColors)[2 * i + 1] = color;
			}
		}
		else
		{
			osgColors = new osg::Vec4Array(1);
			(*osgColors)[0]= osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
			m_lineGeometry->setColorArray(osgColors, osg::Array::BIND_OVERALL);
		}

		// Construct osg lines (start point and end point)
		for (size_t i = 0; i < count; ++i)
		{
			Vector3d point = vertices[i].position;
			(*m_vertexData)[2 * i] = SurgSim::Graphics::toOsg(point);

			(*m_vertexData)[2 * i + 1] = SurgSim::Graphics::toOsg(point) +
									   SurgSim::Graphics::toOsg(vertices[i].data.vectorDirection.getValue())/m_scale;
		}
		m_drawArrays->setCount(count * 2);
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


void OsgVectorFieldRepresentation::setVectorField(std::shared_ptr< SurgSim::Graphics::VectorField > vertices)
{
	m_vertices = vertices;
}


std::shared_ptr< SurgSim::Graphics::VectorField > OsgVectorFieldRepresentation::getVectorField() const
{
	return m_vertices;
}


void OsgVectorFieldRepresentation::setLineWidth(double width)
{
	m_line->setWidth(width);
}

double OsgVectorFieldRepresentation::getLineWidth() const
{
	return static_cast<double>(m_line->getWidth());
}

void OsgVectorFieldRepresentation::setScale(double scale)
{
	m_scale = scale;
}

double OsgVectorFieldRepresentation::getScale() const
{
	return m_scale;
}

}; // Graphics
}; // SurgSim