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

/// Common part of implementation of PlyReaderDelegate for MassSpringRepresentations.
/// This is an abstract class and needs to be inherited.
class MassSpringPlyReaderDelegate : public SurgSim::DataStructures::PlyReaderDelegate
{
public:
	/// Constructor
	MassSpringPlyReaderDelegate();

protected:
	// \return Name of the element (1/2/3D), which this delegate processes.
	virtual std::string getElementName() const = 0;

	bool registerDelegate(SurgSim::DataStructures::PlyReader* reader) override;
	bool fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader) override;

	/// Callback for end of PlyReader::parseFile.
	virtual void endParseFile() = 0;

	/// Callback function, begin the processing of vertices.
	/// \param elementName Name of the element.
	/// \param massCount Number of vertices.
	/// \return memory for mass data to the reader.
	virtual void* beginMasses(const std::string& elementName, size_t massCount);

	/// Callback function to process one mass.
	/// \param elementName Name of the element.
	virtual void processMass(const std::string& elementName) = 0;

	/// Callback function to finalize processing of vertices.
	/// \param elementName Name of the element.
	virtual void endMasses(const std::string& elementName);

	/// Callback function, begin the processing of SpringElements.
	/// \param elementName Name of the element.
	/// \param elementCount Number of elements.
	/// \return memory for SpringElement data to the reader.
	void* beginSpringElement(const std::string& elementName, size_t elementCount);

	/// Callback function to process one SpringElement.
	/// \param elementName Name of the element.
	virtual void processSpringElement(const std::string& elementName) = 0;

	/// Callback function to finalize processing of SpringElements.
	/// \param elementName Name of the element.
	void endSpringElement(const std::string& elementName);
	
	/// Callback function, begin the processing of boundary conditions.
	/// \param elementName Name of the element.
	/// \param boundaryConditionCount Number of boundary conditions.
	/// \return memory for boundary conditions data to the reader.
	void* beginBoundaryConditions(const std::string& elementName, size_t boundaryConditionCount);

	/// Callback function to process one boundary condition.
	/// \param elementName Name of the element.
	virtual void processBoundaryCondition(const std::string& elementName) = 0;

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
	size_t m_boundaryConditionData;

	/// Internal data to receive the spring (stretching and bending) data
	struct SpringData
	{
		int64_t overrun1; ///< Used to check for buffer overruns

		unsigned int* indices;
		unsigned int massCount;
		int64_t overrun2; ///< Used to check for buffer overruns

		double stiffness;
		double damping;
	} m_springData;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGPLYREADERDELEGATE_H
