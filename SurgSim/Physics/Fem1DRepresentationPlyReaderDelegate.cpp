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
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentationPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::DataStructures::PlyReader;

Fem1DRepresentationPlyReaderDelegate::Fem1DRepresentationPlyReaderDelegate(std::shared_ptr<Fem1DRepresentation> fem)
	: FemRepresentationPlyReaderDelegate(fem)
{
}

std::string Fem1DRepresentationPlyReaderDelegate::getElementName() const
{
	return "1d_element";
}

bool Fem1DRepresentationPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	FemRepresentationPlyReaderDelegate::registerDelegate(reader);

	// Radius Processing
	reader->requestElement(
		"radius",
		std::bind(
		&Fem1DRepresentationPlyReaderDelegate::beginRadius, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		nullptr);
	reader->requestScalarProperty("radius", "value", PlyReader::TYPE_DOUBLE, 0);

	return true;
}

bool Fem1DRepresentationPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = FemRepresentationPlyReaderDelegate::fileIsAcceptable(reader);

	result = result && reader.hasProperty("radius", "value");

	return result;
}

void Fem1DRepresentationPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_femData.vertexCount == 2) << "Cannot process 1D Element with "
											   << m_femData.vertexCount << " vertices.";

	std::array<size_t, 2> fem1DVertices;
	std::copy(m_femData.indices, m_femData.indices + 2, fem1DVertices.begin());
	m_fem->addFemElement(std::make_shared<Fem1DElementBeam>(fem1DVertices));
}

void* Fem1DRepresentationPlyReaderDelegate::beginRadius(const std::string& elementName, size_t radiusCount)
{
	return &m_radius;
}

void Fem1DRepresentationPlyReaderDelegate::endParseFile()
{
	for (size_t i = 0; i < m_fem->getNumFemElements(); ++i)
	{
		std::static_pointer_cast<Fem1DElementBeam>(m_fem->getFemElement(i))->setRadius(m_radius);
	}

	FemRepresentationPlyReaderDelegate::endParseFile();
}

}; // namespace Physics
}; // namespace SurgSim