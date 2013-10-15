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

#ifndef SURGSIM_GRAPHICS_OSGSCENERYREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGSCENERYREPRESENTATION_H

#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/SceneryRepresentation.h>

#include <osg/Node>

namespace SurgSim
{

namespace Graphics
{

/// A OsgSceneryRepresentation is used to load osg object/node from file
class OsgSceneryRepresentation:	public OsgRepresentation, public SceneryRepresentation
{
public:
	friend class OsgSceneryRepresentationTest;

	/// Constructor
	/// \param name Name of OsgSceneryRepresentation
	/// \param filePath Path to the file to be loaded
	explicit OsgSceneryRepresentation(const std::string& name);

	/// Return the name of the model of the object
	/// \return Model name of the object
	std::string getModelName() const;
	/// Sets the name of model of the object to be loaded
	/// \param	modelName	Name of the model
	void setModelName(const std::string& modelName);

	/// Return file name of the object
	/// \return File name of the object
	std::string getFileName() const;
	/// Set file name of the object to be loaded
	/// \param	modelName	Name of the model
	void setFileName(const std::string& fileName);

	/// Returns the object
	/// \return A osg Node representing the loaded object
	osg::ref_ptr<osg::Node> getOsgSceneryRepresentation() const;

private:
	virtual bool doInitialize() override;

	/// A osg::Node to hold the objet loaded from file
	osg::ref_ptr<osg::Node> m_sceneryRepresentation;

	/// Name the model/type of the object to be loaded
	std::string m_modelName;
	/// Name of the object file to be loaded
	std::string m_fileName;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGSCENERYREPRESENTATION_H
