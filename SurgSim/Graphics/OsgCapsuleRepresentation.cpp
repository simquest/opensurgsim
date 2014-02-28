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


#include "SurgSim/Graphics/OsgCapsuleRepresentation.h"
#include "SurgSim/Graphics/OsgUnitCylinder.h"
#include "SurgSim/Graphics/OsgUnitSphere.h"

using SurgSim::Graphics::OsgCapsuleRepresentation;
using SurgSim::Graphics::OsgUnitCylinder;
using SurgSim::Graphics::OsgUnitSphere;

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgCapsuleRepresentation);
}

OsgCapsuleRepresentation::OsgCapsuleRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	CapsuleRepresentation(name),
	m_scale(1.0, 1.0),
	m_sharedUnitCylinder(getSharedUnitCylinder()),
	m_sharedUnitSphere(getSharedUnitSphere()),
	m_patCylinder(new osg::PositionAttitudeTransform()),
	m_patSphere1(new osg::PositionAttitudeTransform()),
	m_patSphere2(new osg::PositionAttitudeTransform())
{
	m_patCylinder->addChild(m_sharedUnitCylinder->getNode());
	m_patSphere1->addChild(m_sharedUnitSphere->getNode());
	m_patSphere2->addChild(m_sharedUnitSphere->getNode());

	m_patCylinder->setAttitude(osg::Quat(osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0)));
	m_patSphere1->setPosition(osg::Vec3d(0.0, 0.5, 0.0));
	m_patSphere2->setPosition(osg::Vec3d(0.0, -0.5, 0.0));

	m_transform->addChild(m_patCylinder);
	m_transform->addChild(m_patSphere1);
	m_transform->addChild(m_patSphere2);
}


void OsgCapsuleRepresentation::setRadius(double radius)
{
	m_scale.x() = radius;
	m_patCylinder->setScale(osg::Vec3d(radius, radius, m_scale.y()));
	m_patSphere1->setScale(osg::Vec3d(radius, radius, radius));
	m_patSphere2->setScale(osg::Vec3d(radius, radius, radius));
}
double OsgCapsuleRepresentation::getRadius() const
{
	return m_scale.x();
}

void OsgCapsuleRepresentation::setHeight(double height)
{
	m_scale.y() = height;
	m_patCylinder->setScale(osg::Vec3d(m_scale.x(), m_scale.x(), height));
	m_patSphere1->setPosition(osg::Vec3d(0.0, height / 2, 0.0));
	m_patSphere2->setPosition(osg::Vec3d(0.0, -height / 2, 0.0));
}
double OsgCapsuleRepresentation::getHeight() const
{
	return m_scale.y();
}

void OsgCapsuleRepresentation::setSize(double radius, double height)
{
	m_scale.x() = radius;
	m_scale.y() = height;

	m_patCylinder->setScale(osg::Vec3d(radius, radius, height));
	m_patSphere1->setScale(osg::Vec3d(radius, radius, radius));
	m_patSphere2->setScale(osg::Vec3d(radius, radius, radius));

	m_patSphere1->setPosition(osg::Vec3d(0.0, height / 2, 0.0));
	m_patSphere2->setPosition(osg::Vec3d(0.0, -height / 2, 0.0));
}
void OsgCapsuleRepresentation::getSize(double* radius, double* height)
{
	*radius = m_scale.x();
	*height = m_scale.y();
}

void OsgCapsuleRepresentation::setSize(SurgSim::Math::Vector2d size)
{
	m_scale.set(size.x(), size.y());

	m_patCylinder->setScale(osg::Vec3d(size.x(), size.x(), size.y()));
	m_patSphere1->setScale(osg::Vec3d(size.x(), size.x(), size.x()));
	m_patSphere2->setScale(osg::Vec3d(size.x(), size.x(), size.x()));

	m_patSphere1->setPosition(osg::Vec3d(0.0, size.y() / 2, 0.0));
	m_patSphere2->setPosition(osg::Vec3d(0.0, -size.y() / 2, 0.0));
}
SurgSim::Math::Vector2d OsgCapsuleRepresentation::getSize() const
{
	return SurgSim::Math::Vector2d(m_scale._v);
}

std::shared_ptr<OsgUnitCylinder> OsgCapsuleRepresentation::getSharedUnitCylinder()
{
	static SurgSim::Framework::SharedInstance<OsgUnitCylinder> shared;
	return shared.get();
}

std::shared_ptr<OsgUnitSphere> OsgCapsuleRepresentation::getSharedUnitSphere()
{
	static SurgSim::Framework::SharedInstance<OsgUnitSphere> sharedSphere;
	return sharedSphere.get();
}