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

#ifndef SURGSIM_PHYSICS_FEM3DREPRESENTATIONPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEM3DREPRESENTATIONPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/PlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{

class DeformableRepresentationState;
class Fem3DRepresentation;

/// Implementation of PlyReaderDelegate for simple tetrahedral meshes
class Fem3DRepresentationPlyReaderDelegate : public SurgSim::DataStructures::PlyReaderDelegate
{
public:
	/// Default constructor
	Fem3DRepresentationPlyReaderDelegate();

	/// Gets a shared pointer to the fem.  This class maintains its copy of the pointer until the next time
	/// PlyReader::parseFile is called on the registered PlyReader.
	/// \return The stored fem.
	std::shared_ptr<Fem3DRepresentation> getFem();

	/// Registers the delegate with the reader, overridden from \sa PlyReaderDelegate.
	/// \param reader The reader that should be used.
	/// \return true if it succeeds, false otherwise.
	virtual bool registerDelegate(SurgSim::DataStructures::PlyReader* reader) override;

	/// Check whether this file is acceptable to the delegate, overridden from \sa PlyReaderDelegate.
	/// \param reader The reader that should be used.
	/// \return true if it succeeds, false otherwise.
	virtual bool fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader) override;

	/// Callback for beginning of PlyReader::parseFile.
	void startParseFile();

	/// Callback for end of PlyReader::parseFile.
	void endParseFile();

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

	/// Callback function, begin the processing of tetrahedrons.
	/// \param elementName Name of the element.
	/// \param tetrahedronCount Number of tetrahedrons.
	/// \return memory for tetrahedron data to the reader.
	void* beginTetrahedrons(const std::string& elementName, size_t tetrahedronCount);

	/// Callback function to process one tetrahedron.
	/// \param elementName Name of the element.
	void processTetrahedron(const std::string& elementName);

	/// Callback function to finalize processing of tetrahedrons.
	/// \param elementName Name of the element.
	void endTetrahedrons(const std::string& elementName);

	/// Callback function, begin the processing of boundary conditions.
	/// \param elementName Name of the element.
	/// \param boundaryConditionCount Number of boundary conditions.
	/// \return memory for boundary conditions data to the reader.
	void* beginBoundaryConditions(const std::string& elementName, size_t boundaryConditionCount);

	/// Callback function to process one boundary condition.
	/// \param elementName Name of the element.
	void processBoundaryCondition(const std::string& elementName);

	/// Callback function to finalize processing of boundary conditions.
	/// \param elementName Name of the element.
	void endBoundaryConditions(const std::string& elementName);

private:
	/// Internal data to receive the "vertex" element
	std::array<double, 3> m_vertexData;

	/// Internal iterator to save the "vertex" element
	double *vertexIterator;

	/// Internal data to receive the "tetrahedron" element
	std::array<unsigned int, 4> m_tetrahedronData;

	/// Internal data to receive the "boundary_condition" element
	unsigned int m_boundaryConditionData;

	/// The fem that will be created by loading
	std::shared_ptr<Fem3DRepresentation> m_fem;

	/// The state that will be created by loading
	std::shared_ptr<DeformableRepresentationState> m_state;

	/// Flag indicating whether the associated file has boundary conditions
	bool m_hasBoundaryConditions;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM3DREPRESENTATIONPLYREADERDELEGATE_H
