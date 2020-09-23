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
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/MassSpring.h"

using SurgSim::DataStructures::PlyReader;

namespace SurgSim
{
namespace Physics
{

MassSpringPlyReaderDelegate::MassSpringPlyReaderDelegate()
{
}

MassSpringPlyReaderDelegate::MassSpringPlyReaderDelegate(std::shared_ptr<MassSpring> mesh):
	m_mesh(mesh)
{
	SURGSIM_ASSERT(mesh != nullptr) << "The mesh cannot be null.";
	mesh->clear();
}

bool MassSpringPlyReaderDelegate::registerDelegate(PlyReader* reader)
{
	reader->requestElement("vertex",
						   std::bind(&MassSpringPlyReaderDelegate::beginVertices, this,
									 std::placeholders::_1, std::placeholders::_2),
						   std::bind(&MassSpringPlyReaderDelegate::processVertex, this, std::placeholders::_1),
						   std::bind(&MassSpringPlyReaderDelegate::endVertices, this, std::placeholders::_1));

	reader->requestScalarProperty("vertex", "x", PlyReader::TYPE_DOUBLE, offsetof(MassData, x));
	reader->requestScalarProperty("vertex", "y", PlyReader::TYPE_DOUBLE, offsetof(MassData, y));
	reader->requestScalarProperty("vertex", "z", PlyReader::TYPE_DOUBLE, offsetof(MassData, z));
	reader->requestScalarProperty("vertex", "mass", PlyReader::TYPE_DOUBLE, offsetof(MassData, mass));

	reader->requestElement("1d_element",
		std::bind(&MassSpringPlyReaderDelegate::beginSprings,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		std::bind(&MassSpringPlyReaderDelegate::processSpring, this, std::placeholders::_1),
		std::bind(&MassSpringPlyReaderDelegate::endSprings, this, std::placeholders::_1));

	reader->requestListProperty("1d_element",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(SpringData, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(SpringData, nodeCount));
	reader->requestScalarProperty(
		"1d_element", "stiffness", PlyReader::TYPE_DOUBLE, offsetof(SpringData, stiffness));
	reader->requestScalarProperty(
		"1d_element", "damping", PlyReader::TYPE_DOUBLE, offsetof(SpringData, damping));

	reader->requestElement("bendingSpring",
		std::bind(&MassSpringPlyReaderDelegate::beginSprings,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		std::bind(&MassSpringPlyReaderDelegate::processSpring, this, std::placeholders::_1),
		std::bind(&MassSpringPlyReaderDelegate::endSprings, this, std::placeholders::_1));

	reader->requestListProperty("bendingSpring",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(SpringData, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(SpringData, nodeCount));
	reader->requestScalarProperty(
		"bendingSpring", "stiffness", PlyReader::TYPE_DOUBLE, offsetof(SpringData, stiffness));
	reader->requestScalarProperty(
		"bendingSpring", "damping", PlyReader::TYPE_DOUBLE, offsetof(SpringData, damping));
	
	// Boundary Condition Processing
	if (m_hasBoundaryConditions)
	{
		reader->requestElement("boundary_condition",
			std::bind(&MassSpringPlyReaderDelegate::beginBoundaryConditions,
					  this,
					  std::placeholders::_1,
					  std::placeholders::_2),
			std::bind(&MassSpringPlyReaderDelegate::processBoundaryCondition, this, std::placeholders::_1),
			nullptr);
		reader->requestScalarProperty("boundary_condition", "vertex_index", PlyReader::TYPE_UNSIGNED_INT, 0);
	}

	reader->setEndParseFileCallback(std::bind(&MassSpringPlyReaderDelegate::endParseFile, this));

	return true;
}

bool MassSpringPlyReaderDelegate::fileIsAcceptable(const PlyReader& reader)
{
	bool result = true;

	// Shortcut test if one fails ...
	result = result && reader.hasProperty("vertex", "x");
	result = result && reader.hasProperty("vertex", "y");
	result = result && reader.hasProperty("vertex", "z");
	result = result && reader.hasProperty("vertex", "mass");
	result = result && reader.hasElement("1d_element");
	result = result && reader.hasProperty("1d_element", "vertex_indices");
	result = result && !reader.isScalar("1d_element", "vertex_indices");
	result = result && reader.hasProperty("1d_element", "stiffness") && reader.hasProperty("1d_element", "damping");
	result = result && (!reader.hasElement("bendingSpring") ||
		(reader.hasProperty("bendingSpring", "vertex_indices") && !reader.isScalar("bendingSpring", "vertex_indices") &&
			reader.hasProperty("bendingSpring", "stiffness") && reader.hasProperty("bendingSpring", "damping")));
	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");

	return result;
}

void* MassSpringPlyReaderDelegate::beginVertices(const std::string& elementName, size_t vertexCount)
{
	m_massData.overrun1 = 0l;
	return &m_massData;
}

void MassSpringPlyReaderDelegate::processVertex(const std::string& elementName)
{
	auto mass = std::make_shared<Mass>(m_massData.mass);
	m_mesh->addMass(mass);
	m_mesh->addVertex(DataStructures::Vertices<Mass>::VertexType(
		Math::Vector3d(m_massData.x, m_massData.y, m_massData.z),
		*mass));
}

void MassSpringPlyReaderDelegate::endVertices(const std::string& elementName)
{
	SURGSIM_ASSERT(m_massData.overrun1 == 0l) <<
			"There was an overrun while reading the vertex structures, it is likely that data " <<
			"has become corrupted.";
}

void* MassSpringPlyReaderDelegate::beginSprings(const std::string& elementName, size_t elementCount)
{
	m_springData.overrun1 = 0l;
	m_springData.overrun2 = 0l;
	return &m_springData;
}

void MassSpringPlyReaderDelegate::processSpring(const std::string& elementName)
{
	SURGSIM_ASSERT(m_springData.nodeCount == 2) << "Cannot process spring with " << m_springData.nodeCount << " nodes.";
	auto spring = std::make_shared<LinearSpring>(m_springData.indices[0], m_springData.indices[1]);
	spring->setStiffness(m_springData.stiffness);
	spring->setDamping(m_springData.damping);
	m_mesh->addSpring(spring);
}

void MassSpringPlyReaderDelegate::endSprings(const std::string& elementName)
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

void MassSpringPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_mesh->addBoundaryCondition(static_cast<size_t>(m_boundaryConditionData));
}

void MassSpringPlyReaderDelegate::endParseFile()
{
	m_mesh->update();
}

} // namespace SurgSim
} // namespace DataStructures
