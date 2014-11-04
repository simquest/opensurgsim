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
namespace Framework
{
class Asset;
}

namespace Graphics
{

class Model;

/// Base class defining the interface for a Graphics Scenery Object.
class SceneryRepresentation : public virtual Representation
{
public:

	/// Constructor.
	/// \param	name	The name of the representation.
	explicit SceneryRepresentation(const std::string& name);

	/// Convenience function to trigger the load of the model with the given filename, if successful, this will
	/// replace the old model
	/// \param	fileName Name of the file to be loaded
	virtual void loadModel(const std::string& fileName) = 0;

	/// Set the current model to the model passed
	/// \param model to be used for this scenery representation, this will replace the old model
	virtual void setModel(std::shared_ptr<SurgSim::Framework::Asset> model) = 0;

	/// \return the current model.
	virtual std::shared_ptr<Model> getModel() const = 0;
};

};  // namespace Graphics
};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SCENERYREPRESENTATION_H