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

#include <SurgSim/Graphics/OsgUnitAxes.h>

namespace SurgSim
{
namespace Graphics
{

OsgUnitAxes::OsgUnitAxes()
{
	m_geode = new osg::Geode();
	const int numVertices = 6;
	osg::Geometry* geometry = new osg::Geometry();

	osg::Vec3Array* vertices = new osg::Vec3Array(numVertices);
	(*vertices)[0] = osg::Vec3f(0.0f, 0.0f, 0.0f);
	(*vertices)[1] = osg::Vec3f(0.0f, 0.0f, 0.0f);
	(*vertices)[2] = osg::Vec3f(0.0f, 0.0f, 0.0f);
	(*vertices)[3] = osg::Vec3f(1.0f, 0.0f, 0.0f);
	(*vertices)[4] = osg::Vec3f(0.0f, 1.0f, 0.0f);
	(*vertices)[5] = osg::Vec3f(0.0f, 0.0f, 1.0f);
	geometry->setVertexArray(vertices);

	osg::Vec4Array* colors = new osg::Vec4Array(numVertices);
	(*colors)[0] = osg::Vec4f(0.8f, 0.0f, 0.0f, 1.0f);
	(*colors)[1] = osg::Vec4f(0.0f, 0.8f, 0.0f, 1.0f);
	(*colors)[2] = osg::Vec4f(0.0f, 0.0f, 0.8f, 1.0f);
	(*colors)[3] = osg::Vec4f(0.8f, 0.0f, 0.0f, 1.0f);
	(*colors)[4] = osg::Vec4f(0.0f, 0.8f, 0.0f, 1.0f);
	(*colors)[5] = osg::Vec4f(0.0f, 0.0f, 0.8f, 1.0f);
	geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);

	// Copy triangles from mesh
	unsigned int lines[] = {0,3,1,4,2,5};
	geometry->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,6,lines));

	m_geode->addDrawable(geometry);

	osg::StateSet* state = m_geode->getOrCreateStateSet();
	state->setAttribute(new osg::LineWidth(2.0));

	// These lines should never be lit, always render full color
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

}

OsgUnitAxes::~OsgUnitAxes()
{

}

osg::ref_ptr<osg::Node> OsgUnitAxes::getNode()
{
	return m_geode;
}

}; // Graphics
}; // SurgSim
