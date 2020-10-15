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

#ifndef SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H

#include <array>
#include <memory>
#include <string>

#include "SurgSim/DataStructures/PlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
class MassSpringModel;

/// Common part of implementation of PlyReaderDelegate for MassSpringRepresentations.
/// In order for the same ply file to load with TriangleMeshPlyReaderDelegate
/// (e.g., to create the graphics for a SegmentMesh or to match the shape in a CollisionRep),
/// there must be elements named (for example) 1d_element, that have appropriate vertex_indices.
/// See MassSpringRepresentation::saveMassSpringModel() or MassSpring::saveMassSpringModel() to create a valid ply file.
///
/// Example file that can also be loaded into a SegmentMesh; the third spring resists bending about the middle vertex:
/// ply
/// format ascii 1.0
/// element vertex 3
/// property double x
/// property double y
/// property double z
/// property double mass
/// element 1d_element 2
/// property list uint uint vertex_indices
/// element spring 3
/// property list uint uint vertex_indices
/// property double stiffness
/// property double damping
/// element boundary_condition 0
/// property uint vertex_index
/// element radius 1
/// property double value
/// end_header
/// -0.02 -0.04 -0.001 0.001
/// -0.01 -0.03 -0.00075 0.002
/// 0.0 0.0 0.0 0.002
/// 2 0 1
/// 2 1 2
/// 2 0 1 5 10
/// 2 1 2 5 10
/// 2 0 2 3 10
/// 0.001
///
/// This PlyReaderDelegate can load 1d_element, 2d_element, or 3d_element, passing the vectors of nodes to
/// the MassSpringModel. Similarly, it can load radius or thickness.
class MassSpringPlyReaderDelegate : public SurgSim::DataStructures::PlyReaderDelegate
{
public:
	/// Default constructor
	MassSpringPlyReaderDelegate();

	/// Constructor
	explicit MassSpringPlyReaderDelegate(std::shared_ptr<MassSpringModel> mesh);

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

	/// Callback function, begin the processing of Elements.
	/// \param elementName Name of the element.
	/// \param elementCount Number of elements.
	/// \return memory for Elements data to the reader.
	void* beginElements(const std::string& elementName, size_t elementCount);

	/// Callback function to process one Element.
	/// \param elementName Name of the element.
	virtual void processElement(const std::string& elementName);

	/// Callback function to finalize processing of Elements.
	/// \param elementName Name of the element.
	void endElements(const std::string& elementName);

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

	/// Callback function, begin the processing of radius.
	/// \param elementName Name of the element.
	/// \param radiusCount Number of radii.
	/// \return memory for radius data to the reader.
	void* beginRadius(const std::string& elementName, size_t radiusCount);

	/// Callback function, end the processing of radius.
	/// \param elementName Name of the element.
	void endRadius(const std::string& elementName);

	/// Callback function, begin the processing of thickness.
	/// \param elementName Name of the element.
	/// \param thicknessCount Number of radii.
	/// \return memory for thickness data to the reader.
	void* beginThickness(const std::string& elementName, size_t thicknessCount);

	/// Callback function, end the processing of thickness.
	/// \param elementName Name of the element.
	void endThickness(const std::string& elementName);

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

	/// Flag indicating if the associated file has 1d elements
	bool m_has1dElement = false;
	
	/// Flag indicating if the associated file has 2d elements
	bool m_has2dElement = false;

	/// Flag indicating if the associated file has 3d elements
	bool m_has3dElement = false;

	/// Internal data to receive the element (nodeId) data
	struct ElementData
	{
		int64_t overrun1; ///< Used to check for buffer overruns
		unsigned int* indices;
		unsigned int nodeCount;
		int64_t overrun2; ///< Used to check for buffer overruns
	} m_elementData;

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

	/// Flag indicating if the associated file has a radius
	bool m_hasRadius;
	/// Radius information, if any.
	double m_radius;

	/// Flag indicating if the associated file has a thickness
	bool m_hasThickness;
	/// Thickness information, if any.
	double m_thickness;

	/// MassSpringModel to contain the ply file information
	std::shared_ptr<MassSpringModel> m_mesh;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H
