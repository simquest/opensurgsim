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

#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/StateAttribute>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"

namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgPointCloudRepresentation,
				 OsgPointCloudRepresentation);

OsgPointCloudRepresentation::OsgPointCloudRepresentation(const std::string& name) :
	Representation(name),
	PointCloudRepresentation(name),
	OsgRepresentation(name),
	m_color(1.0, 1.0, 1.0, 1.0)
{
	m_vertices = std::make_shared<PointCloud>();

	osg::Geode* geode = new osg::Geode();
	m_geometry = new osg::Geometry();
	m_vertexData = new osg::Vec3Array;

	m_geometry->setVertexArray(m_vertexData);

	setColor(m_color);

	// At this stage there are no vertices in there
	m_drawArrays = new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,m_vertexData->size());
	m_geometry->addPrimitiveSet(m_drawArrays);
	m_geometry->setUseDisplayList(false);
	// m_geometry->setUseVertexBufferObjects(true);
	m_geometry->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	m_point = new osg::Point(1.0f);
	m_geometry->getOrCreateStateSet()->setAttribute( m_point, osg::StateAttribute::ON );

	geode->addDrawable(m_geometry);
	m_transform->addChild(geode);
}


OsgPointCloudRepresentation::~OsgPointCloudRepresentation()
{
}

void OsgPointCloudRepresentation::doUpdate(double dt)
{
	auto& vertices = m_vertices->getVertices();
	size_t count = vertices.size();

	// Check for size change in number of vertices
	if (count != static_cast<size_t>(m_drawArrays->getCount()))
	{
		m_drawArrays->setCount(count);
		if (count > m_vertexData->size())
		{
			m_vertexData->resize(count);
		}

		m_drawArrays->set(osg::PrimitiveSet::POINTS,0,count);
		m_drawArrays->dirty();
	}

	for (size_t i = 0; i < count; ++i)
	{
		(*m_vertexData)[i][0] = static_cast<float>(vertices[i].position[0]);
		(*m_vertexData)[i][1] = static_cast<float>(vertices[i].position[1]);
		(*m_vertexData)[i][2] = static_cast<float>(vertices[i].position[2]);
	}

	m_geometry->dirtyBound();
	m_geometry->dirtyDisplayList();
}

std::shared_ptr<PointCloud> OsgPointCloudRepresentation::getVertices() const
{
	return m_vertices;
}

void OsgPointCloudRepresentation::setPointSize(double val)
{
	m_point->setSize(val);
}

double OsgPointCloudRepresentation::getPointSize() const
{
	return static_cast<double>(m_point->getSize());
}

void OsgPointCloudRepresentation::setColor(const SurgSim::Math::Vector4d& color)
{
	// Set the color of the particles to one single color by default
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

SurgSim::Math::Vector4d OsgPointCloudRepresentation::getColor() const
{
	return m_color;
}

}; // Graphics
}; // SurgSim
