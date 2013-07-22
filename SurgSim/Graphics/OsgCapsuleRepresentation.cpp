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



#include <SurgSim/Graphics/OsgCapsuleRepresentation.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>
#include <SurgSim/Graphics/OsgUnitCylinder.h>
#include <SurgSim/Graphics/OsgUnitSphere.h>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

using SurgSim::Graphics::OsgCapsuleRepresentation;
using SurgSim::Graphics::OsgUnitCylinder;
using SurgSim::Graphics::OsgUnitSphere;


OsgCapsuleRepresentation::OsgCapsuleRepresentation(const std::string& name) :
	Representation(name),
	CapsuleRepresentation(name),
	OsgRepresentation(name),
	m_scale(1.0, 1.0),
	m_sharedUnitCylinder(getSharedUnitCylinder()),
	m_sharedUnitSphere1(getSharedUnitSphere1()),
	m_sharedUnitSphere2(getSharedUnitSphere2()),
	m_PatCylinder(new osg::PositionAttitudeTransform()),
	m_PatSphere1(new osg::PositionAttitudeTransform()),
	m_PatSphere2(new osg::PositionAttitudeTransform())
{
	m_PatCylinder->addChild(m_sharedUnitCylinder->getNode());
	m_PatSphere1->addChild(m_sharedUnitSphere1->getNode());
	m_PatSphere2->addChild(m_sharedUnitSphere2->getNode());
	m_PatCylinder->setAttitude(osg::Quat(osg::PI_2, osg::Vec3d(1.0, 0.0, 0.0)));
	m_PatSphere1->setPosition(osg::Vec3d(0.0, 0.5, 0.0));
	m_PatSphere2->setPosition(osg::Vec3d(0.0, -0.5, 0.0));
	m_transform->addChild(m_PatCylinder);
	m_transform->addChild(m_PatSphere1);
	m_transform->addChild(m_PatSphere2);
}


void OsgCapsuleRepresentation::setRadius(double radius)
{
	m_scale.x() = radius;
	m_PatCylinder->setScale(osg::Vec3d(radius, radius, m_scale.y()));
	m_PatSphere1->setScale(osg::Vec3d(radius, radius, radius));
	m_PatSphere2->setScale(osg::Vec3d(radius, radius, radius));
}
double OsgCapsuleRepresentation::getRadius() const
{
	return m_scale.x();
}

void OsgCapsuleRepresentation::setHeight(double height)
{
	m_scale.y() = height;
	m_PatCylinder->setScale(osg::Vec3d(m_scale.x(), m_scale.x(), height));
	m_PatSphere1->setPosition(osg::Vec3d(0.0, height/2, 0.0));
	m_PatSphere2->setPosition(osg::Vec3d(0.0, -height/2, 0.0));
}
double OsgCapsuleRepresentation::getHeight() const
{
	return m_scale.y();
}

void OsgCapsuleRepresentation::setSize(double radius, double height)
{
	m_scale.x() = radius;
	m_scale.y() = height;
	m_PatCylinder->setScale(osg::Vec3d(radius, radius, height));
	m_PatSphere1->setScale(osg::Vec3d(radius, radius, radius));
	m_PatSphere2->setScale(osg::Vec3d(radius, radius, radius));
	m_PatSphere1->setPosition(osg::Vec3d(0.0, height/2, 0.0));
	m_PatSphere2->setPosition(osg::Vec3d(0.0, -height/2, 0.0));
}
void OsgCapsuleRepresentation::getSize(double* radius, double* height)
{
	*radius = m_scale.x();
	*height = m_scale.y();
}

void OsgCapsuleRepresentation::setSize(SurgSim::Math::Vector2d size)
{
	m_scale.set(size.x(), size.y());
	m_PatCylinder->setScale(osg::Vec3d(size.x(), size.x(), size.y()));
	m_PatSphere1->setScale(osg::Vec3d(size.x(), size.x(), size.x()));
	m_PatSphere2->setScale(osg::Vec3d(size.x(), size.x(), size.x()));
	m_PatSphere1->setPosition(osg::Vec3d(0.0, size.y()/2, 0.0));
	m_PatSphere2->setPosition(osg::Vec3d(0.0, -size.y()/2, 0.0));
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

std::shared_ptr<OsgUnitSphere> OsgCapsuleRepresentation::getSharedUnitSphere1()
{
	static SurgSim::Framework::SharedInstance<OsgUnitSphere> sharedSphere1;
	return sharedSphere1.get();
}

std::shared_ptr<OsgUnitSphere> OsgCapsuleRepresentation::getSharedUnitSphere2()
{
	static SurgSim::Framework::SharedInstance<OsgUnitSphere> sharedSphere2;
	return sharedSphere2.get();
}