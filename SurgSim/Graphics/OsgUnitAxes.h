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

#ifndef SURGSIM_GRAPHICS_OSGUNITAXES_H
#define SURGSIM_GRAPHICS_OSGUNITAXES_H

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/Vec3f>
#include <osg/LineWidth>

namespace SurgSim
{
namespace Graphics
{

class OsgUnitAxes
{
public:

	/// Constructor
	OsgUnitAxes();
	~OsgUnitAxes();

	/// Gets the node.
	/// \return	The node.
	osg::ref_ptr<osg::Node> getNode();

private:
	osg::ref_ptr<osg::Geode> m_geode;
};

}; // Graphics
}; // SurgSim

#endif