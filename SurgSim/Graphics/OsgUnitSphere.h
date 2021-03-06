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

#ifndef SURGSIM_GRAPHICS_OSGUNITSPHERE_H
#define SURGSIM_GRAPHICS_OSGUNITSPHERE_H

#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/Quat>
#include <osg/Shape>
#include <osg/ShapeDrawable>

namespace SurgSim
{

namespace Graphics
{

/// OSG unit sphere geode to be used as a primitive shape
/// The sphere is located at (0, 0, 0) and has a radius of 1.
/// Add the sphere geode to a transform node to position and scale it.
class OsgUnitSphere
{
public:
	/// Constructor
	OsgUnitSphere() :
		m_transform(new osg::PositionAttitudeTransform())
	{
		osg::ref_ptr<osg::Sphere> unitSphere = new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), 1.0);
		osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(unitSphere);
		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
		geode->addDrawable(drawable);
		geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
		m_transform->addChild(geode);
		osg::Quat rotation;
		rotation.makeRotate(osg::Vec3d(0.0, 0.0, 1.0), osg::Vec3d(0.0, 1.0, 0.0));
		m_transform->setAttitude(rotation);
	}

	/// Returns the root OSG node for the plane to be inserted into the scene-graph
	osg::ref_ptr<osg::Node> getNode() const
	{
		return m_transform;
	}

private:
	/// Root OSG node of the sphere
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGUNITSPHERE_H
