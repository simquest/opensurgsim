// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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
#include "SurgSim/Math/Valid.h"
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
	m_mesh(mesh), m_hasBoundaryConditions(false), m_hasRadius(false), m_hasThickness(false)
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
		std::bind(&MassSpringPlyReaderDelegate::beginElements,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		std::bind(&MassSpringPlyReaderDelegate::processElement, this, std::placeholders::_1),
		std::bind(&MassSpringPlyReaderDelegate::endElements, this, std::placeholders::_1));

	reader->requestListProperty("1d_element",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(ElementData, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(ElementData, nodeCount));

	reader->requestElement("spring",
		std::bind(&MassSpringPlyReaderDelegate::beginSprings,
			this,
			std::placeholders::_1,
			std::placeholders::_2),
		std::bind(&MassSpringPlyReaderDelegate::processSpring, this, std::placeholders::_1),
		std::bind(&MassSpringPlyReaderDelegate::endSprings, this, std::placeholders::_1));

	reader->requestListProperty("spring",
		"vertex_indices",
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(SpringData, indices),
		PlyReader::TYPE_UNSIGNED_INT,
		offsetof(SpringData, nodeCount));
	reader->requestScalarProperty(
		"spring", "stiffness", PlyReader::TYPE_DOUBLE, offsetof(SpringData, stiffness));
	reader->requestScalarProperty(
		"spring", "damping", PlyReader::TYPE_DOUBLE, offsetof(SpringData, damping));
	
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

	if (m_hasRadius)
	{
		// Radius Processing
		reader->requestElement(
			"radius",
			std::bind(
				&MassSpringPlyReaderDelegate::beginRadius, this, std::placeholders::_1, std::placeholders::_2),
			nullptr,
			std::bind(&MassSpringPlyReaderDelegate::endRadius, this, std::placeholders::_1));
		reader->requestScalarProperty("radius", "value", PlyReader::TYPE_DOUBLE, 0);
	}

	if (m_hasThickness)
	{
		// Thickness Processing
		reader->requestElement(
			"thickness",
			std::bind(
				&MassSpringPlyReaderDelegate::beginThickness, this, std::placeholders::_1, std::placeholders::_2),
			nullptr,
			std::bind(&MassSpringPlyReaderDelegate::endThickness, this, std::placeholders::_1));
		reader->requestScalarProperty("thickness", "value", PlyReader::TYPE_DOUBLE, 0);
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

	// The file can have at most one of the three types of elements, though they are treated equivalently.
	result = result && ((!reader.hasElement("1d_element") && !reader.hasElement("2d_element")) ||
		(!reader.hasElement("1d_element") && !reader.hasElement("3d_element")) ||
		(!reader.hasElement("2d_element") && !reader.hasElement("3d_element")));

	result = result && (!reader.hasElement("1d_element")) ||
		(reader.hasProperty("1d_element", "vertex_indices") && !reader.isScalar("1d_element", "vertex_indices"));
	result = result && (!reader.hasElement("2d_element")) ||
		(reader.hasProperty("2d_element", "vertex_indices") && !reader.isScalar("2d_element", "vertex_indices"));
	result = result && (!reader.hasElement("3d_element")) ||
		(reader.hasProperty("3d_element", "vertex_indices") && !reader.isScalar("3d_element", "vertex_indices"));

	result = result && reader.hasElement("spring") && reader.hasProperty("spring", "vertex_indices") &&
		!reader.isScalar("spring", "vertex_indices") && reader.hasProperty("spring", "stiffness") &&
		reader.hasProperty("spring", "damping");
	m_hasBoundaryConditions = reader.hasProperty("boundary_condition", "vertex_index");
	m_hasRadius = reader.hasProperty("radius", "value");
	m_hasThickness = reader.hasProperty("thickness", "value");

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

void* MassSpringPlyReaderDelegate::beginElements(const std::string & elementName, size_t elementCount)
{
	m_elementData.overrun1 = 0l;
	m_elementData.overrun2 = 0l;
	return &m_elementData;
}

void MassSpringPlyReaderDelegate::processElement(const std::string & elementName)
{
	SURGSIM_ASSERT(m_elementData.nodeCount > 0) << "Cannot process element with 0 nodes.";
	std::vector<size_t> nodeIds(m_elementData.indices, m_elementData.indices + m_elementData.nodeCount);
	for (const auto& id : nodeIds)
	{
		SURGSIM_ASSERT(m_mesh->getNumVertices() > id) << "processElement was given a element with a nodeId (" <<
			id << ") that is not in the vertices.";
	}
	m_mesh->addElement(nodeIds);
}

void MassSpringPlyReaderDelegate::endElements(const std::string & elementName)
{
	SURGSIM_ASSERT(m_elementData.overrun1 == 0 && m_elementData.overrun2 == 0) <<
		"There was an overrun while reading the element structures, it is likely that data " <<
		"has become corrupted.";
	m_elementData.indices = nullptr;
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
	SURGSIM_ASSERT(m_mesh->getNumVertices() > m_springData.indices[0]) <<
		"processSpring was given a spring with a nodeId (" << m_springData.indices[0] <<
		") that is not in the vertices.";
	SURGSIM_ASSERT(m_mesh->getNumVertices() > m_springData.indices[1]) <<
		"processSpring was given a spring with a nodeId (" << m_springData.indices[1] <<
		") that is not in the vertices.";
	m_mesh->addSpring(std::make_shared<LinearSpring>(m_mesh, m_springData.indices[0], m_springData.indices[1],
		m_springData.stiffness, m_springData.damping));
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

void* MassSpringPlyReaderDelegate::beginRadius(const std::string& elementName, size_t radiusCount)
{
	return &m_radius;
}

void MassSpringPlyReaderDelegate::endRadius(const std::string& elementName)
{
	SURGSIM_ASSERT(SurgSim::Math::isValid(m_radius)) << "No radius information processed.";
	m_mesh->setRadius(m_radius);
}

void* MassSpringPlyReaderDelegate::beginThickness(const std::string& elementName, size_t thicknessCount)
{
	return &m_thickness;
}

void MassSpringPlyReaderDelegate::endThickness(const std::string& elementName)
{
	SURGSIM_ASSERT(SurgSim::Math::isValid(m_thickness)) << "No thickness information processed.";
	m_mesh->setThickness(m_thickness);
}

void MassSpringPlyReaderDelegate::endParseFile()
{
	m_mesh->update();
}

} // namespace SurgSim
} // namespace DataStructures
