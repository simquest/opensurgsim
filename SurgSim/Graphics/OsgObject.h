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

#ifndef SURGSIM_GRAPHICS_OSGOBJECT_H
#define SURGSIM_GRAPHICS_OSGOBJECT_H

#include <SurgSim/Graphics/Object.h>

#include <osg/Object>

namespace SurgSim
{

namespace Graphics
{

class OsgObject : public SurgSim::Graphics::Object
{
public:
	/// Constructor
	OsgObject();

	/// Loads an osg object from a file
	/// \param	filePath	Path to the object file
	/// \return	True if the object is successfully loaded, otherwise false
	virtual bool loadObject(const std::string& filePath) override;

	/// Release any held object
	virtual void unloadObject() override;

	/// Returns the object
	osg::ref_ptr<osg::Object> getOsgObject() const;

private:
	/// OSG object
	osg::ref_ptr<osg::Object> m_object;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGOBJECT_H
