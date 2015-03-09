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

#include "SurgSim/Graphics/OsgRepresentation.h"

#include <algorithm>

#include <boost/thread/locks.hpp>

#include <osg/Geode>
#include <osg/Switch>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgUnitBox.h"
#include "SurgSim/Graphics/TangentSpaceGenerator.h"

namespace SurgSim
{
namespace Graphics
{

OsgRepresentation::OsgRepresentation(const std::string& name) :
	Representation(name),
	m_drawAsWireFrame(false)
{
	m_switch = new osg::Switch;
	m_switch->setName(name + " Representation Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_transform->setName(name + " Transform");

	m_switch->addChild(m_transform);

	m_transform->setAttitude(osg::Quat(0.0, 0.0, 0.0, 1.0));
	m_transform->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
}

OsgRepresentation::~OsgRepresentation()
{

}

void OsgRepresentation::update(double dt)
{

	if (isActive())
	{
		std::pair<osg::Quat, osg::Vec3d> pose = toOsg(getPose());
		m_transform->setAttitude(pose.first);
		m_transform->setPosition(pose.second);
		doUpdate(dt);
		setVisible(true);
	}
	else
	{
		setVisible(false);
	}

}

bool OsgRepresentation::setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
{
	bool didSucceed = false;

	std::shared_ptr<OsgMaterial> osgMaterial = std::dynamic_pointer_cast<OsgMaterial>(material);
	if (osgMaterial != nullptr)
	{
		m_transform->setStateSet(osgMaterial->getOsgStateSet());
		didSucceed = true;
		m_material = osgMaterial;
	}
	return didSucceed;
}

std::shared_ptr<Material> OsgRepresentation::getMaterial() const
{
	return m_material;
}

void OsgRepresentation::clearMaterial()
{
	m_transform->setStateSet(new osg::StateSet()); // Reset to empty state set
	m_material = nullptr;
}

osg::ref_ptr<osg::Node> OsgRepresentation::getOsgNode() const
{
	return m_switch;
}

void OsgRepresentation::doUpdate(double dt)
{

}

void OsgRepresentation::setDrawAsWireFrame(bool val)
{
	m_drawAsWireFrame = val;
	osg::StateSet* state = m_switch->getOrCreateStateSet();

	osg::ref_ptr<osg::PolygonMode> polygonMode;
	if (val)
	{
		polygonMode = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	}
	else
	{
		polygonMode = new osg::PolygonMode(osg::PolygonMode::FRONT, osg::PolygonMode::FILL);
	}

	state->setAttributeAndModes(polygonMode, osg::StateAttribute::ON);
}

bool OsgRepresentation::getDrawAsWireFrame() const
{
	return m_drawAsWireFrame;
}

void OsgRepresentation::setVisible(bool val)
{
	m_switch->setChildValue(m_transform, val);
}



void OsgRepresentation::setGenerateTangents(bool value)
{
	if (value && m_tangentGenerator == nullptr)
	{
		m_tangentGenerator =
			new TangentSpaceGenerator(DIFFUSE_TEXTURE_UNIT, TANGENT_VERTEX_ATTRIBUTE_ID, BITANGENT_VERTEX_ATTRIBUTE_ID);
		m_tangentGenerator->setBasisOrthonormality(true);
		updateTangents();
	}
	if (!value)
	{
		m_tangentGenerator = nullptr;
	}
}

bool OsgRepresentation::isGeneratingTangents() const
{
	return m_tangentGenerator != nullptr;
}

void OsgRepresentation::updateTangents()
{
	if (m_tangentGenerator != nullptr)
	{
		m_switch->accept(*m_tangentGenerator);
	}
}

}; // Graphics
}; // SurgSim
