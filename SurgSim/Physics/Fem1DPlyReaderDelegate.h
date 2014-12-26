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

#ifndef SURGSIM_PHYSICS_FEM1DPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEM1DPLYREADERDELEGATE_H

#include <memory>
#include <string>

#include "SurgSim/Physics/FemPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
class Fem1DRepresentation;

/// Implementation of PlyReaderDelegate for Fem1DRepresentation
class Fem1DPlyReaderDelegate : public SurgSim::Physics::FemPlyReaderDelegate
{
public:
	/// Constructor
	/// \param fem The object that is updated when PlyReader::parseFile is called.
	explicit Fem1DPlyReaderDelegate(std::shared_ptr<Fem1DRepresentation> fem);

protected:
	std::string getElementName() const override;

	bool registerDelegate(SurgSim::DataStructures::PlyReader* reader) override;
	bool fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader) override;

	void endParseFile() override;
	void processFemElement(const std::string& elementName) override;

	/// Callback function, begin the processing of radius.
	/// \param elementName Name of the element.
	/// \param radiusCount Number of radii.
	/// \return memory for radius data to the reader.
	void* beginRadius(const std::string& elementName, size_t radiusCount);

	/// Callback function, end the processing of radius.
	/// \param elementName Name of the element.
	void endRadius(const std::string& elementName);

private:
	double m_radius;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM1DPLYREADERDELEGATE_H