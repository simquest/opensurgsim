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

#include <array>

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

Fem3DPlyReaderDelegate::Fem3DPlyReaderDelegate(std::shared_ptr<Fem3DRepresentation> fem)
	: FemPlyReaderDelegate(fem)
{
}

std::string Fem3DPlyReaderDelegate::getElementName() const
{
	return "3d_element";
}

void Fem3DPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(4== m_femData.vertexCount || 8 == m_femData.vertexCount) <<
		"Cannot process 3D element with " << m_femData.vertexCount << " vertices.";

	if (4 == m_femData.vertexCount)
	{
		std::array<size_t, 4> fem3DVertices;
		std::copy(m_femData.indices, m_femData.indices + 4, fem3DVertices.begin());
		m_fem->addFemElement(std::make_shared<Fem3DElementTetrahedron>(fem3DVertices));
	}
	else
	{
		std::array<size_t, 8> fem3DVertices;
		std::copy(m_femData.indices, m_femData.indices + 8, fem3DVertices.begin());
		m_fem->addFemElement(std::make_shared<Fem3DElementCube>(fem3DVertices));
	}
}

}; // namespace Physics
}; // namespace SurgSim