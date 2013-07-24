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

#include <SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h>
#include <SurgSim/Graphics/View.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Projection>
#include <osg/StateAttribute>
#include <osg/Switch>

namespace SurgSim
{
namespace Graphics
{

OsgScreenSpaceQuadRepresentation::OsgScreenSpaceQuadRepresentation(
		const std::string& name,
		std::shared_ptr<View> view) :
	Representation(name),
	OsgRepresentation(name),
	ScreenSpaceQuadRepresentation(name,view),
	m_view(view),
	m_scale(1.0,1.0,1.0)
{
	// get rid of the transform we want to set this up ourselves
	m_switch->removeChild(0u,1u);

	m_view->getDimensions(&m_displayWidth, &m_displayHeight);

	m_geode = new osg::Geode;

	// Make the quad
	osg::Geometry* geom = new osg::Geometry;

	osg::Vec3Array* m_vertices = new osg::Vec3Array;
	float depth = 0.0;
	m_vertices->push_back(osg::Vec3(0.0,1.0,depth));
	m_vertices->push_back(osg::Vec3(0.0,0.0,depth));
	m_vertices->push_back(osg::Vec3(1.0,0.0,depth));
	m_vertices->push_back(osg::Vec3(1.0,1.0,depth));
	setSize(1.0,1.0);

	geom->setVertexArray(m_vertices);

	osg::Vec3Array* normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
	geom->setNormalArray(normals);
	geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.2f));
	geom->setColorArray(colors);
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

	osg::StateSet* stateset = geom->getOrCreateStateSet();
	stateset->setMode(GL_BLEND,osg::StateAttribute::ON);

	m_geode->addDrawable(geom);

	m_transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	m_transform->setCullingActive(false);
	m_transform->addChild(m_geode);

	// Need to keep this around to modify the ortho projection when the view changes
	m_projection = new osg::Projection;
	m_projection->setMatrix(osg::Matrix::ortho2D(0,m_displayWidth,0,m_displayHeight));
	m_projection->addChild(m_transform);

	m_switch->addChild(m_projection);
}

OsgScreenSpaceQuadRepresentation::~OsgScreenSpaceQuadRepresentation()
{

}

void OsgScreenSpaceQuadRepresentation::doUpdate(double dt)
{
	int width, height;
	m_view->getDimensions(&width, &height);
	if (width != m_displayWidth || height != m_displayHeight)
	{
		m_displayWidth = width;
		m_displayHeight = height;
		m_projection->setMatrix(osg::Matrix::ortho2D(0, m_displayWidth, 0, m_displayHeight));
	}
}

void OsgScreenSpaceQuadRepresentation::setSize(int width, int height)
{
	m_scale.x() = width;
	m_scale.y() = height;
	m_transform->setScale(m_scale);
}

void OsgScreenSpaceQuadRepresentation::getSize(int* width, int* height) const
{
	*width = static_cast<int>(m_scale.x());
	*height = static_cast<int>(m_scale.y());
}

void OsgScreenSpaceQuadRepresentation::setPose(const SurgSim::Math::RigidTransform3d& transform)
{
	// HS-2013-jun-28 This function should probably be protected by a mutes, but I can see
	// the assumption could be that this is only called from on thread.
	// #threadsafety
	m_pose = transform;
	std::pair<osg::Quat, osg::Vec3d> pose = toOsg(m_pose);
	m_transform->setPosition(pose.second);
}

}; // Graphics
}; // SurgSim
