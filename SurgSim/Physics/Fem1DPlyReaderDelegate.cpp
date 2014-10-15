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
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"

namespace SurgSim
{
namespace Physics
{
using SurgSim::DataStructures::PlyReader;

Fem1DPlyReaderDelegate::Fem1DPlyReaderDelegate(std::shared_ptr<Fem1DRepresentation> fem) :
	FemPlyReaderDelegate(fem),
	m_radius(std::numeric_limits<double>::quiet_NaN())
{
}

std::string Fem1DPlyReaderDelegate::getElementName() const
{
	return "1d_element";
}

bool Fem1DPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	FemPlyReaderDelegate::registerDelegate(reader);

	// Radius Processing
	reader->requestElement(
		"radius",
		std::bind(
		&Fem1DPlyReaderDelegate::beginRadius, this, std::placeholders::_1, std::placeholders::_2),
		nullptr,
		std::bind(&Fem1DPlyReaderDelegate::endRadius, this, std::placeholders::_1));
	reader->requestScalarProperty("radius", "value", PlyReader::TYPE_DOUBLE, 0);

	return true;
}

bool Fem1DPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = FemPlyReaderDelegate::fileIsAcceptable(reader);
	result = result && reader.hasProperty("radius", "value");

	return result;
}

void Fem1DPlyReaderDelegate::processFemElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_femData.vertexCount == 2) << "Cannot process 1D Element with "
											   << m_femData.vertexCount << " vertices.";

	std::array<size_t, 2> fem1DVertices;
	std::copy(m_femData.indices, m_femData.indices + 2, fem1DVertices.begin());
	m_fem->addFemElement(std::make_shared<Fem1DElementBeam>(fem1DVertices));
}

void* Fem1DPlyReaderDelegate::beginRadius(const std::string& elementName, size_t radiusCount)
{
	return &m_radius;
}

void Fem1DPlyReaderDelegate::endRadius(const std::string& elementName)
{
	SURGSIM_ASSERT(SurgSim::Math::isValid(m_radius)) << "No radius information processed.";
}

void Fem1DPlyReaderDelegate::endParseFile()
{
	for (size_t i = 0; i < m_fem->getNumFemElements(); ++i)
	{
		std::static_pointer_cast<Fem1DElementBeam>(m_fem->getFemElement(i))->setRadius(m_radius);
	}

	FemPlyReaderDelegate::endParseFile();
}

}; // namespace Physics
}; // namespace SurgSim