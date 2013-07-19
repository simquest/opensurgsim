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
//#include <SurgSim/Graphics/OsgUnitCapsule.h>
#include <SurgSim/Graphics/OsgUnitCylinder.h>
#include <SurgSim/Graphics/OsgUnitSphere.h>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

using SurgSim::Graphics::OsgCapsuleRepresentation;
//using SurgSim::Graphics::OsgUnitCapsule;
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
	//m_sharedUnitCapsule(getSharedUnitCapsule())
{
	//m_transform->addChild(m_sharedUnitCapsule->getNode());
	m_PatCylinder->addChild(m_sharedUnitCylinder->getNode());
	m_PatSphere1->addChild(m_sharedUnitSphere1->getNode());
	m_PatSphere2->addChild(m_sharedUnitSphere2->getNode());
	m_transform->addChild(m_PatCylinder);
	m_transform->addChild(m_PatSphere1);
	m_transform->addChild(m_PatSphere2);
}


void OsgCapsuleRepresentation::setRadius(double radius)
{
	m_scale.x() = radius;
	//m_transform->setScale(osg::Vec3d(radius, m_scale.y(), radius));
	m_PatCylinder->setScale(osg::Vec3d(radius, m_scale.y(), radius));
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
	osg::Vec3d sphere1OldPosition = m_PatSphere1->getPosition();
	osg::Vec3d sphere2OldPosition = m_PatSphere2->getPosition();

	//m_transform->setScale(osg::Vec3d(m_scale.x(), height, m_scale.x()));
	m_PatCylinder->setScale(osg::Vec3d(m_scale.x(), height, m_scale.x()));

	sphere1OldPosition.y() += height/2;
	sphere2OldPosition.y() -= height/2;
	m_PatSphere1->setPosition(sphere1OldPosition);
	m_PatSphere2->setPosition(sphere2OldPosition);
}
double OsgCapsuleRepresentation::getHeight() const
{
	return m_scale.y();
}

void OsgCapsuleRepresentation::setSize(double radius, double height)
{
	m_scale.x() = radius;
	m_scale.y() = height;
	osg::Vec3d sphere1OldPosition = m_PatSphere1->getPosition();
	osg::Vec3d sphere2OldPosition = m_PatSphere2->getPosition();	
	
	//m_transform->setScale(osg::Vec3d(radius, height, radius));
	m_PatCylinder->setScale(osg::Vec3d(radius, height, radius));
	m_PatSphere1->setScale(osg::Vec3d(radius, radius, radius));
	m_PatSphere2->setScale(osg::Vec3d(radius, radius, radius));
	
	sphere1OldPosition.y() += height/2;
	sphere2OldPosition.y() -= height/2;
	m_PatSphere1->setPosition(sphere1OldPosition);
	m_PatSphere2->setPosition(sphere2OldPosition);
}
void OsgCapsuleRepresentation::getSize(double* radius, double* height)
{
	*radius = m_scale.x();
	*height = m_scale.y();
}

void OsgCapsuleRepresentation::setSize(SurgSim::Math::Vector2d size)
{
	m_scale.set(size.x(), size.y());
	osg::Vec3d sphere1OldPosition = m_PatSphere1->getPosition();
	osg::Vec3d sphere2OldPosition = m_PatSphere2->getPosition();

	//m_transform->setScale(osg::Vec3d(size.x(), size.y(), size.x()));
	m_PatCylinder->setScale(osg::Vec3d(size.x(), size.y(), size.x()));
	m_PatSphere1->setScale(osg::Vec3d(size.x(), size.x(), size.x()));
	m_PatSphere2->setScale(osg::Vec3d(size.x(), size.x(), size.x()));

	sphere1OldPosition.y() += size.y()/2;
	sphere2OldPosition.y() -= size.y()/2;
	m_PatSphere1->setPosition(sphere1OldPosition);
	m_PatSphere2->setPosition(sphere2OldPosition);
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