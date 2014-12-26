// This file is a part of the OpenSurgSim project.
// Copyright 2014, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_FEM3DPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEM3DPLYREADERDELEGATE_H

#include <memory>

#include "SurgSim/Physics/FemPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
class Fem3DRepresentation;

/// Implementation of PlyReaderDelegate for Fem3DRepresentation
class Fem3DPlyReaderDelegate : public FemPlyReaderDelegate
{
public:
	/// Constructor
	/// \param fem The object that is updated when PlyReader::parseFile is called.
	explicit Fem3DPlyReaderDelegate(std::shared_ptr<Fem3DRepresentation> fem);

protected:
	std::string getElementName() const override;

	void processFemElement(const std::string& elementName) override;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DPLYREADERDELEGATE_H