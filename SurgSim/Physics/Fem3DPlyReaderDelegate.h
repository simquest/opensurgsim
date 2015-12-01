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

#ifndef SURGSIM_PHYSICS_FEM3DPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEM3DPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Physics/Fem3D.h"
#include "SurgSim/Physics/FemPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{

class Fem3DPlyReaderDelegate : public FemPlyReaderDelegate
{
public:
	/// Default constructor.
	Fem3DPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	explicit Fem3DPlyReaderDelegate(std::shared_ptr<Fem3D> mesh);

protected:
	std::string getElementName() const override;

	bool fileIsAcceptable(const PlyReader& reader) override;

	void endParseFile() override;

	void processVertex(const std::string& elementName) override;

	void processFemElement(const std::string& elementName) override;

	void processBoundaryCondition(const std::string& elementName) override;

	/// End file callback
	void endFile();

private:
	/// Fem3D mesh asset to contain the ply file information
	std::shared_ptr<Fem3D> m_mesh;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DPLYREADERDELEGATE_H
