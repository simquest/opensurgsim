// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_MASSSPRING1DPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_MASSSPRING1DPLYREADERDELEGATE_H

#include "SurgSim/Physics/MassSpringPlyReaderDelegate.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"

#include <vector>

namespace SurgSim
{
namespace Physics
{

class MassSpring1DPlyReaderDelegate : public MassSpringPlyReaderDelegate
{
public:
	/// Default constructor.
	MassSpring1DPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	explicit MassSpring1DPlyReaderDelegate(MassSpringRepresentation* massSpring);

protected:
	std::string getElementName() const override;
	void endParseFile() override;
	void processMass(const std::string& elementName) override;
	void processSpringElement(const std::string& elementName) override;
	void processBoundaryCondition(const std::string& elementName) override;

private:
	std::vector<size_t> m_nodeBoundaryConditions;
	std::vector<MassSpringRepresentation::MassElement> m_masses;
	std::vector<MassSpringRepresentation::SpringElement> m_springs;

	/// MassSpringRepresentation to contain the ply file information
	MassSpringRepresentation* m_massSpring;
};


} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRING1DPLYREADERDELEGATE_H
