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

#ifndef SURGSIM_GRAPHICS_SCENERYREPRESENTATION_H
#define SURGSIM_GRAPHICS_SCENERYREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

#include <string>

namespace SurgSim
{

namespace Graphics
{

/// Base class defining the interface for a Graphics Scenery Object.
class SceneryRepresentation : public virtual Representation
{
public:

	/// Constructor.
	/// \param	name	The name of the representation.
	explicit SceneryRepresentation(const std::string& name): Representation(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(SceneryRepresentation, std::string, FileName, getFileName, setFileName);
	}

	/// Return file name of the object
	/// \return File name of the object
	virtual std::string getFileName() const = 0;

	/// Set file name of the object to be loaded
	/// \param	fileName Name of the file to be loaded
	virtual void setFileName(const std::string& fileName) = 0;
};

};  // namespace Graphics
};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SCENERYREPRESENTATION_H