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

#ifndef SURGSIM_GRAPHICS_OSGPLANE_H
#define SURGSIM_GRAPHICS_OSGPLANE_H

#include <osg/Geode>
#include <osg/Geometry>

namespace SurgSim
{

namespace Graphics
{

/// OSG plane geode to be used as a primitive shape
/// The plane is the XZ plane, with normal +Y.
/// The plane is drawn with a Quad at (0.0, 0.0, 0.0) with length and width specified in the constructor (or default of
/// 1000 for each).
/// Add the plane geode to a transform node to position it.
class OsgPlane
{
public:
	/// Constructor
	/// \param	length	Length of the plane in X (default is 1000)
	/// \param	width	Width of the plane in Z (default is 1000)
	explicit OsgPlane(float length = 1000.0f, float width = 1000.0f) :
		m_geode(new osg::Geode())
	{
		osg::ref_ptr<osg::Geometry> plane = osg::createTexturedQuadGeometry(
			osg::Vec3(- length / 2.0f, 0.0f, width / 2.0f),
			osg::Vec3(length, 0.0f, 0.0f),
			osg::Vec3(0.0f, 0.0f, - width));
		/// Normal is X^-Z = Y
		m_geode->addDrawable(plane);
	}

	/// Returns the root OSG node for the plane to be inserted into the scene-graph
	osg::ref_ptr<osg::Node> getNode() const
	{
		return m_geode;
	}

private:
	/// Root OSG node of the plane
	osg::ref_ptr<osg::Geode> m_geode;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGPLANE_H
