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
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/StateAttribute>

#include <SurgSim/Graphics/OsgConversions.h>

namespace SurgSim
{
namespace Graphics
{

using SurgSim::DataStructures::Vertex;

OsgVectorFieldRepresentation::OsgVectorFieldRepresentation(const std::string& name) :
	Representation(name),
	VectorFieldRepresentation(name),
	OsgRepresentation(name),
	m_vectorField(nullptr),
	m_scale(10.0)
{
	m_vertexData = new osg::Vec3Array;
	m_lineGeometry = new osg::Geometry;
	m_pointGeometry = new osg::Geometry;
	m_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINES);
	m_drawPoints = new osg::DrawElementsUInt(osg::PrimitiveSet::POINTS);
	m_line = new osg::LineWidth(1.0f);
	m_point = new osg::Point(4.0f);
	m_colors = new osg::Vec4Array;

	m_lineGeometry->setVertexArray(m_vertexData);
	m_lineGeometry->addPrimitiveSet(m_drawArrays);
	m_lineGeometry->setUseDisplayList(false);
	m_lineGeometry->setDataVariance(osg::Object::DYNAMIC);
	m_lineGeometry->getOrCreateStateSet()->setAttribute(m_line, osg::StateAttribute::ON);

	m_pointGeometry->setVertexArray(m_vertexData);
	m_pointGeometry->addPrimitiveSet(m_drawPoints);
	m_pointGeometry->setUseDisplayList(false);
	m_pointGeometry->setDataVariance(osg::Object::DYNAMIC);
	m_pointGeometry->getOrCreateStateSet()->setAttribute(m_point, osg::StateAttribute::ON);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(m_lineGeometry);
	geode->addDrawable(m_pointGeometry);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
	m_transform->addChild(geode);
}


OsgVectorFieldRepresentation::~OsgVectorFieldRepresentation()
{
}

void OsgVectorFieldRepresentation::doUpdate(double dt)
{
	std::vector< Vertex<VectorFieldData> > vertices = m_vectorField->getVertices();
	size_t count = vertices.size();

	// Update vertices information to be used in OSG
	for (size_t i = 0; i < count; ++i)
	{
		// Starting location of vector
		(*m_vertexData)[2 * i] = SurgSim::Graphics::toOsg(vertices[i].position);
		// Ending location of vector
		(*m_vertexData)[2 * i + 1] = SurgSim::Graphics::toOsg(vertices[i].position) +
			SurgSim::Graphics::toOsg(vertices[i].data.direction) / m_scale;
	}

	if (count > m_colors->size() * 2)
	{
		m_colors->resize(count * 2);
	}
	if (vertices[0].data.color.hasValue())
	{
		for (size_t i = 0; i < count; ++i)
		{
			osg::Vec4d color = SurgSim::Graphics::toOsg(vertices[i].data.color.getValue());
			(*m_colors)[2 * i] = color;
			(*m_colors)[2 * i + 1] = color;
		}
		m_lineGeometry->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
		m_pointGeometry->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
	}
	else
	{
		m_colors = new osg::Vec4Array(1);
		(*m_colors)[0]= osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
		m_lineGeometry->setColorArray(m_colors, osg::Array::BIND_OVERALL);
		m_pointGeometry->setColorArray(m_colors, osg::Array::BIND_OVERALL);
	}
}


bool OsgVectorFieldRepresentation::setVectorField(std::shared_ptr< SurgSim::Graphics::VectorField > vectorField)
{
	if (! isAwake())
	{
		m_vectorField = vectorField;
		size_t count = m_vectorField->getVertices().size();

		m_drawArrays->setCount(count * 2);
		m_drawArrays->dirty();

		m_vertexData->resize(count * 2);

		m_drawPoints->dirty();
		m_drawPoints->resize(count);
		for (size_t i = 0; i < count; ++i)
		{
			m_drawPoints->at(i) = (2 * i);
		}
		return true;
	}
	else
	{
		return false;
	}
}


std::shared_ptr< SurgSim::Graphics::VectorField > OsgVectorFieldRepresentation::getVectorField() const
{
	return m_vectorField;
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

void OsgVectorFieldRepresentation::setPointSize(double size)
{
	m_point->setSize(size);
}

double OsgVectorFieldRepresentation::getPointSize() const
{
	return static_cast<double>(m_point->getSize());
}

}; // Graphics
}; // SurgSim