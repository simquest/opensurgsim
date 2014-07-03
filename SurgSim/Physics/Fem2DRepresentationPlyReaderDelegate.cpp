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
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem2DRepresentationPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::DataStructures::PlyReader;

Fem2DRepresentationPlyReaderDelegate::Fem2DRepresentationPlyReaderDelegate(std::shared_ptr<Fem2DRepresentation> fem)
	: FemRepresentationPlyReaderDelegate(fem)
{
}

std::string Fem2DRepresentationPlyReaderDelegate::getElementName() const
{
	return "2d_element";
}

bool Fem2DRepresentationPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	FemRepresentationPlyReaderDelegate::registerDelegate(reader);

	// Thickness Processing
	reader->requestElement(
		"thickness",
		std::bind(
		&Fem2DRepresentationPlyReaderDelegate::beginThickness, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		nullptr);
	reader->requestScalarProperty("thickness", "value", PlyReader::TYPE_DOUBLE, 0);

	return true;
}

bool Fem2DRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = FemRepresentationPlyReaderDelegate::fileIsAcceptable(reader);

	result = result && reader.hasProperty("thickness", "value");

	return result;
}

void Fem2DRepresentationPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_femData.vertexCount == 3) << "Cannot process 2D Element with "
											   << m_femData.vertexCount << " vertices.";

	std::array<size_t, 3> fem2DVertices;
	std::copy(m_femData.indices, m_femData.indices + 3, fem2DVertices.begin());
	m_fem->addFemElement(std::make_shared<Fem2DElementTriangle>(fem2DVertices));
}

void* Fem2DRepresentationPlyReaderDelegate::beginThickness(const std::string& elementName, size_t thicknessCount)
{
	return &m_thickness;
}

void Fem2DRepresentationPlyReaderDelegate::endParseFile()
{
	for (size_t i = 0; i < m_fem->getNumFemElements(); ++i)
	{
		std::static_pointer_cast<Fem2DElementTriangle>(m_fem->getFemElement(i))->setThickness(m_thickness);
	}

	FemRepresentationPlyReaderDelegate::endParseFile();
}

}; // namespace Physics
}; // namespace SurgSim