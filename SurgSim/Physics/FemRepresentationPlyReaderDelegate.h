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

#ifndef SURGSIM_PHYSICS_FEMREPRESENTATIONPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEMREPRESENTATIONPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/PlyReaderDelegate.h"

namespace SurgSim
{

namespace Math
{
class OdeState;
};

namespace Physics
{
class FemRepresentation;

/// Common part of implementation of PlyReaderDelegate for FemRepresentations.
/// This is an abstract class and needs to be inherited.
/// Methods 'registerDelegate()' and 'fileIsAcceptable()' need to be overridden.
class FemRepresentationPlyReaderDelegate : public SurgSim::DataStructures::PlyReaderDelegate
{
public:
	/// Constructor
	/// \param fem The object that is updated when PlyReader::parseFile is called.
	explicit FemRepresentationPlyReaderDelegate(std::shared_ptr<FemRepresentation> fem);

	/// Callback for beginning of PlyReader::parseFile.
	void startParseFile();

	/// Callback for end of PlyReader::parseFile.
	virtual void endParseFile();

	/// Callback function, begin the processing of vertices.
	/// \param elementName Name of the element.
	/// \param vertexCount Number of vertices.
	/// \return memory for vertex data to the reader.
	void* beginVertices(const std::string& elementName, size_t vertexCount);

	/// Callback function to process one vertex.
	/// \param elementName Name of the element.
	void processVertex(const std::string& elementName);

	/// Callback function to finalize processing of vertices.
	/// \param elementName Name of the element.
	void endVertices(const std::string& elementName);

	/// Callback function, begin the processing of FemElements.
	/// \param elementName Name of the element.
	/// \param elementCount Number of elements.
	/// \return memory for FemElement data to the reader.
	void* beginFemElements(const std::string& elementName, size_t elementCount);

	/// Callback function to process one FemElement.
	/// \param elementName Name of the element.
	virtual void processFemElement(const std::string& elementName) = 0;

	/// Callback function to finalize processing of FemElements.
	/// \param elementName Name of the element.
	void endFemElements(const std::string& elementName);

	/// Callback function, begin the processing of materials.
	/// \param elementName Name of the element.
	/// \param materialCount Number of materials.
	/// \return memory for material data to the reader.
	void* beginMaterials(const std::string& elementName, size_t materialCount);

	/// Callback function, begin the processing of boundary conditions.
	/// \param elementName Name of the element.
	/// \param boundaryConditionCount Number of boundary conditions.
	/// \return memory for boundary conditions data to the reader.
	void* beginBoundaryConditions(const std::string& elementName, size_t boundaryConditionCount);

	/// Callback function to process one boundary condition.
	/// \param elementName Name of the element.
	void processBoundaryCondition(const std::string& elementName);

protected:
	/// Flag indicating if the associated file has boundary conditions
	bool m_hasBoundaryConditions;

	/// Internal data to receive the "boundary_condition" element
	size_t m_boundaryConditionData;

	/// Internal iterator to save the "vertex" element
	double* m_vertexIterator;

	/// Internal data to receive the "vertex" element
	std::array<double, 3> m_vertexData;

	/// The fem that will be created by loading
	std::shared_ptr<FemRepresentation> m_fem;

	/// The state that will be created by loading
	std::shared_ptr<SurgSim::Math::OdeState> m_state;

	/// Internal data to receive the "material" data
	struct MaterialData
	{
		double massDensity;
		double poissonRatio;
		double youngModulus;
	} m_materialData;

	/// Internal data to receive the fem element
	struct ElementData
	{
		ElementData();

		unsigned int* indices;
		unsigned int vertexCount;
	} m_femData;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMREPRESENTATIONPLYREADERDELEGATE_H