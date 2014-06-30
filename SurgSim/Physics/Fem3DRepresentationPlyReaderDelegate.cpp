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
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

Fem3DRepresentationPlyReaderDelegate::Fem3DRepresentationPlyReaderDelegate(std::shared_ptr<Fem3DRepresentation> fem)
	: FemRepresentationPlyReaderDelegate(fem)
{
}

bool Fem3DRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");

	result = result && reader.hasProperty("3d_element", "vertex_indices");
	result = result && !reader.isScalar("3d_element", "vertex_indices");

	result = result && reader.hasProperty("material", "mass_density");
	result = result && reader.hasProperty("material", "poisson_ratio");
	result = result && reader.hasProperty("material", "young_modulus");

	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");

	return result;
}

bool Fem3DRepresentationPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	FemRepresentationPlyReaderDelegate::registerDelegate(reader);

	// 3D Element Processing
	reader->requestElement(
		"3d_element",
		std::bind(&Fem3DRepresentationPlyReaderDelegate::beginFemElements,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::processFemElement, this, std::placeholders::_1),
		std::bind(&Fem3DRepresentationPlyReaderDelegate::endFemElements, this, std::placeholders::_1));
	reader->requestListProperty("3d_element",
								"vertex_indices",
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(ElementData, indices),
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(ElementData, vertexCount));

	return true;
}

void Fem3DRepresentationPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_femData.vertexCount == 4) << "Cannot process polyhedron with "
											   << m_femData.vertexCount << " vertices.";

	std::array<size_t, 4> polyhedronVertices;
	std::copy(m_femData.indices, m_femData.indices + 4, polyhedronVertices.begin());
	m_fem->addFemElement(std::make_shared<Fem3DElementTetrahedron>(polyhedronVertices));
}

}; // namespace Physics
}; // namespace SurgSim