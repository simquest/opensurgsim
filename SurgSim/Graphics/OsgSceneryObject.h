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

#ifndef SURGSIM_GRAPHICS_OSGSCENERYOBJECT_H
#define SURGSIM_GRAPHICS_OSGSCENERYOBJECT_H

#include <SurgSim/Graphics/OsgRepresentation.h>

#include <osg/Node>

namespace SurgSim
{

namespace Graphics
{

/// A OsgSceneryObject is used to load osg object/node from file
class OsgSceneryObject :
	public OsgRepresentation
{
public:
	friend class OsgSceneryObjectTest;

	/// Constructor
	/// \param name Name of OsgSceneryObject
	/// \param filePath Path to the file to be loaded
	explicit OsgSceneryObject(const std::string& name, const std::string& filePath = "");

	/// Returns the object
	/// \return A osg Node representing the loaded object
	osg::ref_ptr<osg::Node> getOsgSceneryObject() const;

private:
	virtual bool doInitialize() override;

	/// OSG object
	osg::ref_ptr<osg::Node> m_sceneryObject;

	/// Path of the object file to be loaded
	std::string m_filePath;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGSCENERYOBJECT_H
