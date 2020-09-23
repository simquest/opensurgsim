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

#ifndef SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/PlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
class MassSpring;

/// Common part of implementation of PlyReaderDelegate for MassSpringRepresentations.
/// In order for the same ply file to load with TriangleMeshPlyReaderDelegate
/// (e.g., to create the graphics for a SegmentMesh), there must be elements named (for example) 1d_element,
/// that have appropriate vertex_indices.  This class reuses those elements to hold springs
/// (which can have 0 stiffness and damping). Springs that are not between adjacent vertices are held in
/// "bendingSpring" elements.
class MassSpringPlyReaderDelegate : public SurgSim::DataStructures::PlyReaderDelegate
{
public:
	/// Default constructor
	MassSpringPlyReaderDelegate();

	/// Constructor
	explicit MassSpringPlyReaderDelegate(std::shared_ptr<MassSpring> mesh);

protected:
	bool registerDelegate(SurgSim::DataStructures::PlyReader* reader) override;
	bool fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader) override;

	/// Callback for end of PlyReader::parseFile.
	virtual void endParseFile();

	/// Callback function, begin the processing of vertices.
	/// \param elementName Name of the element.
	/// \param vertexCount Number of vertices.
	/// \return memory for mass data to the reader.
	virtual void* beginVertices(const std::string& elementName, size_t vertexCount);

	/// Callback function to process one vertex.
	/// \param elementName Name of the element.
	virtual void processVertex(const std::string& elementName);

	/// Callback function to finalize processing of vertices.
	/// \param elementName Name of the element.
	virtual void endVertices(const std::string& elementName);

	/// Callback function, begin the processing of Springs.
	/// \param elementName Name of the element.
	/// \param elementCount Number of elements.
	/// \return memory for Springs data to the reader.
	void* beginSprings(const std::string& elementName, size_t elementCount);

	/// Callback function to process one Spring.
	/// \param elementName Name of the element.
	virtual void processSpring(const std::string& elementName);

	/// Callback function to finalize processing of Springs.
	/// \param elementName Name of the element.
	void endSprings(const std::string& elementName);
	
	/// Callback function, begin the processing of boundary conditions.
	/// \param elementName Name of the element.
	/// \param boundaryConditionCount Number of boundary conditions.
	/// \return memory for boundary conditions data to the reader.
	void* beginBoundaryConditions(const std::string& elementName, size_t boundaryConditionCount);

	/// Callback function to process one boundary condition.
	/// \param elementName Name of the element.
	virtual void processBoundaryCondition(const std::string& elementName);

protected:
	/// Mass data containing 3 translational dofs and mass
	struct MassData
	{
		double x;
		double y;
		double z;
		double mass;
		int64_t overrun1; ///< Used to check for buffer overruns
	} m_massData;

	/// Flag indicating if the associated file has boundary conditions
	bool m_hasBoundaryConditions;

	/// Internal data to receive the "boundary_condition" element
	unsigned int m_boundaryConditionData;

	/// Internal data to receive the spring (stretching and bending) data
	struct SpringData
	{
		int64_t overrun1; ///< Used to check for buffer overruns
		unsigned int* indices;
		unsigned int nodeCount;
		int64_t overrun2; ///< Used to check for buffer overruns
		double stiffness;
		double damping;
	} m_springData;

	/// MassSpring to contain the ply file information
	std::shared_ptr<MassSpring> m_mesh;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H
