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

#ifndef EXAMPLES_STAPLING_STAPLEELEMENT_H
#define EXAMPLES_STAPLING_STAPLEELEMENT_H

#include <string>

#include "SurgSim/Framework/BasicSceneElement.h"

namespace SurgSim
{
namespace Graphics
{
class Material;
}
}

class StapleElement : public SurgSim::Framework::BasicSceneElement
{
public:

	/// Constructor
	/// \param name Name of the staple element.
	explicit StapleElement(const std::string& name);

	/// Specify whether the staple was created with a collision representation.
	/// \param flag Flag to specify whether the staple was created with a collision representation.
	void setHasCollisionRepresentation(bool flag);

protected:
	/// Initialize this scene element
	/// \return True on success, otherwise false.
	bool doInitialize() override;

private:
	std::shared_ptr<SurgSim::Graphics::Material> findMaterial(
		const std::string& elementName,
		const std::string& materialName);


	/// Flag to specify if the stapleElement was created with a collision representation.
	bool m_hasCollisionRepresentation;

};

#endif //EXAMPLES_STAPLING_STAPLEELEMENT_H
