// This file is a part of the OpenSurgSim project.
// Copyright 2014-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/MassSpringPlyReaderDelegate.h"

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

MassSpringPlyReaderDelegate::MassSpringPlyReaderDelegate()
{
}

bool MassSpringPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	// Element Processing
	reader->requestElement(
		getElementName(),
		std::bind(&MassSpringPlyReaderDelegate::beginSpringElement,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2),
		std::bind(&MassSpringPlyReaderDelegate::processSpringElement, this, std::placeholders::_1),
		std::bind(&MassSpringPlyReaderDelegate::endSpringElement, this, std::placeholders::_1));

	reader->requestScalarProperty(
		getElementName(), "stiffness", PlyReader::TYPE_DOUBLE, offsetof(SpringData, stiffness));
	reader->requestScalarProperty(
		getElementName(), "damping", PlyReader::TYPE_DOUBLE, offsetof(SpringData, damping));

	reader->requestListProperty(getElementName(),
								"mass_indices",
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(SpringData, indices),
								PlyReader::TYPE_UNSIGNED_INT,
								offsetof(SpringData, massCount));

	// Mass processing
	reader->requestElement("mass",
						   std::bind(&MassSpringPlyReaderDelegate::beginMasses, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&MassSpringPlyReaderDelegate::processMass, this, std::placeholders::_1),
						   std::bind(&MassSpringPlyReaderDelegate::endMasses, this, std::placeholders::_1));

	reader->requestScalarProperty("mass", "x", PlyReader::TYPE_DOUBLE, offsetof(MassData, x));
	reader->requestScalarProperty("mass", "y", PlyReader::TYPE_DOUBLE, offsetof(MassData, y));
	reader->requestScalarProperty("mass", "z", PlyReader::TYPE_DOUBLE, offsetof(MassData, z));
	reader->requestScalarProperty("mass", "mass", PlyReader::TYPE_DOUBLE, offsetof(MassData, mass));
	
	// Boundary Condition Processing
	if (m_hasBoundaryConditions)
	{
		reader->requestElement(
			"boundary_condition",
			std::bind(&MassSpringPlyReaderDelegate::beginBoundaryConditions,
					  this,
					  std::placeholders::_1,
					  std::placeholders::_2),
			std::bind(&MassSpringPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "mass_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	reader->setEndParseFileCallback(std::bind(&MassSpringPlyReaderDelegate::endParseFile, this));

	return true;
}

bool MassSpringPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasElement(getElementName());

	result = result && reader.hasProperty("mass", "x");
	result = result && reader.hasProperty("mass", "y");
	result = result && reader.hasProperty("mass", "z");

	result = result && reader.hasProperty(getElementName(), "mass_indices");
	result = result && !reader.isScalar(getElementName(), "mass_indices");

	result = result && reader.hasProperty(getElementName(), "stiffness") &&
		reader.hasProperty(getElementName(), "damping");
	
	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "mass_index");

	return result;
}

void* MassSpringPlyReaderDelegate::beginMasses(const std::string& elementName, size_t massCount)
{
	m_massData.overrun1 = 0l;
	return &m_massData;
}

void MassSpringPlyReaderDelegate::endMasses(const std::string& elementName)
{
	SURGSIM_ASSERT(m_massData.overrun1 == 0l) <<
			"There was an overrun while reading the mass structures, it is likely that data " <<
			"has become corrupted.";
}

void* MassSpringPlyReaderDelegate::beginSpringElement(const std::string& elementName, size_t elementCount)
{
	m_springData.overrun1 = 0l;
	m_springData.overrun2 = 0l;
	return &m_springData;
}

void MassSpringPlyReaderDelegate::endSpringElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_springData.overrun1 == 0 && m_springData.overrun2 == 0) <<
			"There was an overrun while reading the spring structures, it is likely that data " <<
			"has become corrupted.";
	m_springData.indices = nullptr;
}

void* MassSpringPlyReaderDelegate::beginBoundaryConditions(const std::string& elementName,
		size_t boundaryConditionCount)
{
	return &m_boundaryConditionData;
}

} // namespace SurgSim
} // namespace DataStructures
