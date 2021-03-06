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

#ifndef SURGSIM_GRAPHICS_OSGUNITBOX_H
#define SURGSIM_GRAPHICS_OSGUNITBOX_H

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

namespace SurgSim
{

namespace Graphics
{

/// OSG unit box geode to be used as a primitive shape
/// The box is located at (0, 0, 0) and has a size of 1 on all three axes.
/// Add the box geode to a transform node to position and scale it.
class OsgUnitBox
{
public:
	/// Constructor
	OsgUnitBox() :
		m_geode(new osg::Geode())
	{
		osg::ref_ptr<osg::Box> unitBox = new osg::Box(osg::Vec3(0.0, 0.0, 0.0), 1.0);
		osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(unitBox);
		m_geode->addDrawable(drawable);
		m_geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	}

	/// Returns the root OSG node for the plane to be inserted into the scene-graph
	osg::ref_ptr<osg::Node> getNode() const
	{
		return m_geode;
	}

private:
	/// Root OSG node of the box
	osg::ref_ptr<osg::Geode> m_geode;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGUNITBOX_H
