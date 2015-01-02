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

#ifndef SURGSIM_PHYSICS_FEM2DPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEM2DPLYREADERDELEGATE_H

#include <memory>

#include "SurgSim/Physics/FemPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
class Fem2DRepresentation;

/// Implementation of PlyReaderDelegate for Fem2DRepresentation
class Fem2DPlyReaderDelegate : public SurgSim::Physics::FemPlyReaderDelegate
{
public:
	/// Constructor
	/// \param fem The object that is updated when PlyReader::parseFile is called.
	explicit Fem2DPlyReaderDelegate(std::shared_ptr<Fem2DRepresentation> fem);

protected:
	std::string getElementName() const override;

	bool registerDelegate(SurgSim::DataStructures::PlyReader* reader) override;
	bool fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader) override;

	void endParseFile() override;
	void processFemElement(const std::string& elementName) override;

	/// Callback function, begin the processing of thickness.
	/// \param elementName Name of the element.
	/// \param thicknessCount Number of thicknesses.
	/// \return memory for thickness data to the reader.
	void* beginThickness(const std::string& elementName, size_t thicknessCount);

	/// Callback function, end the processing of thickness.
	/// \param elementName Name of the element.
	void endThickness(const std::string& elementName);

private:
	double m_thickness;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM2DPLYREADERDELEGATE_H