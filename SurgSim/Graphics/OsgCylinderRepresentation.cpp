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

#include "SurgSim/Graphics/OsgCylinderRepresentation.h"

#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgUnitCylinder.h"

#include "SurgSim/Framework/SharedInstance.h"

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgCylinderRepresentation,
				 OsgCylinderRepresentation);

OsgCylinderRepresentation::OsgCylinderRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	CylinderRepresentation(name),
	m_scale(1.0, 1.0),
	m_sharedUnitCylinder(getSharedUnitCylinder()),
	m_patCylinder(new osg::PositionAttitudeTransform)
{
	m_patCylinder->addChild(m_sharedUnitCylinder->getNode());
	m_patCylinder->setAttitude(osg::Quat(osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0)));
	m_transform->addChild(m_patCylinder);
}


void OsgCylinderRepresentation::setRadius(double radius)
{
	m_scale.x() = radius;
	m_patCylinder->setScale(osg::Vec3d(radius, radius, m_scale.y()));
}
double OsgCylinderRepresentation::getRadius() const
{
	return m_scale.x();
}

void OsgCylinderRepresentation::setHeight(double height)
{
	m_scale.y() = height;
	m_patCylinder->setScale(osg::Vec3d(m_scale.x(), m_scale.x(), height));
}
double OsgCylinderRepresentation::getHeight() const
{
	return m_scale.y();
}

void OsgCylinderRepresentation::setSize(double radius, double height)
{
	m_scale.x() = radius;
	m_scale.y() = height;
	m_patCylinder->setScale(osg::Vec3d(radius, radius, height));
}
void OsgCylinderRepresentation::getSize(double* radius, double* height)
{
	*radius = m_scale.x();
	*height = m_scale.y();
}

void OsgCylinderRepresentation::setSize(const SurgSim::Math::Vector2d& size)
{
	m_scale.set(size.x(), size.y());
	m_patCylinder->setScale(osg::Vec3d(size.x(), size.x(), size.y()));
}
SurgSim::Math::Vector2d OsgCylinderRepresentation::getSize() const
{
	return SurgSim::Math::Vector2d(m_scale._v);
}

std::shared_ptr<OsgUnitCylinder> OsgCylinderRepresentation::getSharedUnitCylinder()
{
	static SurgSim::Framework::SharedInstance<OsgUnitCylinder> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim