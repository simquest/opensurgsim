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

#include "SurgSim/Graphics/OsgCurveRepresentation.h"

#include <array>

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Hint>
#include <osg/LineWidth>
#include <osg/PositionAttitudeTransform>

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Math/CardinalSplines.h"
#include "SurgSim/Math/Geometry.h"

namespace SurgSim
{

namespace Graphics
{

OsgCurveRepresentation::OsgCurveRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	CurveRepresentation(name),
	m_subdivision(100),
	m_tension(0.4)
{
	osg::Geode* geode = new osg::Geode();
	m_geometry = new osg::Geometry();
	m_geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// At this stage there are no vertices in there
	m_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 0);
	m_geometry->addPrimitiveSet(m_drawArrays);
	m_geometry->setUseDisplayList(false);
	m_geometry->setUseVertexBufferObjects(true);
	m_geometry->setDataVariance(osg::Object::DYNAMIC);

	m_vertexData = new osg::Vec3Array;
	m_geometry->setVertexArray(m_vertexData);

	m_normalData = new osg::Vec3Array;
	m_geometry->setNormalArray(m_normalData, osg::Array::BIND_PER_VERTEX);

	setColor(Math::Vector4d(1.0, 1.0, 1.0, 1.0));
	setWidth(1.5);
	setAntiAliasing(true);

	geode->addDrawable(m_geometry);
	m_transform->addChild(geode);
}

OsgCurveRepresentation::~OsgCurveRepresentation()
{
}

bool OsgCurveRepresentation::doInitialize()
{
	return true;
}

bool OsgCurveRepresentation::doWakeUp()
{
	return true;
}

void OsgCurveRepresentation::doUpdate(double dt)
{
	DataStructures::VerticesPlain controlPoints;
	if (m_locker.tryTakeChanged(&controlPoints))
	{
		updateGraphics(controlPoints);
	}
}

void OsgCurveRepresentation::setSubdivisions(size_t num)
{
	m_subdivision = num;
}

size_t OsgCurveRepresentation::getSubdivisions() const
{
	return m_subdivision;
}

void OsgCurveRepresentation::setTension(double tension)
{
	m_tension = tension;
}

double OsgCurveRepresentation::getTension() const
{
	return m_tension;
}

void OsgCurveRepresentation::updateGraphics(const DataStructures::VerticesPlain& controlPoints)
{
	const double stepsize = 1.0 / (m_subdivision + 1);

	Math::CardinalSplines::extendControlPoints(controlPoints, &m_controlPoints);

	size_t numPoints = static_cast<size_t>(static_cast<double>(m_controlPoints.size() - 3) / stepsize);

	m_vertices.clear();
	m_vertices.reserve(numPoints);

	Math::CardinalSplines::interpolate(getSubdivisions(), m_controlPoints, &m_vertices, getTension());

	size_t vertexCount = m_vertices.size();
	if (m_vertexData->size() != vertexCount)
	{
		m_vertexData->resize(vertexCount);
		m_normalData->resize(vertexCount);

		m_drawArrays->set(osg::PrimitiveSet::LINE_STRIP, 0, vertexCount);
		m_drawArrays->dirty();
	}

	// #performance
	// Calculate the bounding box while iterating over the vertices, this will save osg time in the update traversal
	for (size_t i = 0; i < vertexCount; ++i)
	{
		const auto& vertex0 = m_vertices[i];
		const auto& vertex1 = (i < vertexCount - 1) ? m_vertices[i + 1] : m_controlPoints.back();

		// Assign the segment into the normal for use in the shader
		const auto& normal = vertex1 - vertex0;

		(*m_normalData)[i][0] = static_cast<float>(normal[0]);
		(*m_normalData)[i][1] = static_cast<float>(normal[1]);
		(*m_normalData)[i][2] = static_cast<float>(normal[2]);

		(*m_vertexData)[i][0] = static_cast<float>(vertex0[0]);
		(*m_vertexData)[i][1] = static_cast<float>(vertex0[1]);
		(*m_vertexData)[i][2] = static_cast<float>(vertex0[2]);
	}

	m_vertexData->dirty();
	m_normalData->dirty();
	m_geometry->dirtyBound();
}

void OsgCurveRepresentation::setWidth(double width)
{
	auto lineWidth = new osg::LineWidth(width);
	m_geometry->getOrCreateStateSet()->setAttribute(lineWidth, osg::StateAttribute::ON);
	m_width = width;
}

double OsgCurveRepresentation::getWidth() const
{
	return m_width;
}

void OsgCurveRepresentation::setAntiAliasing(bool val)
{
	auto state = m_geometry->getOrCreateStateSet();
	if (val && state->getMode(GL_BLEND) == osg::StateAttribute::INHERIT)
	{
		state->setMode(GL_BLEND, osg::StateAttribute::ON);
		state->setMode(GL_LINE_SMOOTH, osg::StateAttribute::ON);
	}
	else
	{
		state->setMode(GL_BLEND, osg::StateAttribute::INHERIT);
		state->setMode(GL_LINE_SMOOTH, osg::StateAttribute::INHERIT);
	}
}

bool OsgCurveRepresentation::isAntiAliasing() const
{
	return m_geometry->getOrCreateStateSet()->getMode(GL_BLEND) == osg::StateAttribute::ON;
}

void OsgCurveRepresentation::setColor(const SurgSim::Math::Vector4d& color)
{
	osg::Vec4Array* colors = dynamic_cast<osg::Vec4Array*>(m_geometry->getColorArray());
	if (colors == nullptr)
	{
		colors = new osg::Vec4Array(1);
		m_geometry->setColorArray(colors);
		m_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	}
	(*colors)[0] = SurgSim::Graphics::toOsg(color);
	m_color = color;
}

Math::Vector4d OsgCurveRepresentation::getColor() const
{
	return m_color;
}

}
}
