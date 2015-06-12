// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_FEM1DPLYREADERDELEGATE_H
#define SURGSIM_PHYSICS_FEM1DPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Physics/Fem1D.h"
#include "SurgSim/Physics/FemPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{

class Fem1DPlyReaderDelegate : public FemPlyReaderDelegate
{
public:
	/// Default constructor.
	Fem1DPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	explicit Fem1DPlyReaderDelegate(std::shared_ptr<Fem1D> mesh);

protected:
	std::string getElementName() const override;

	bool registerDelegate(PlyReader* reader) override;

	bool fileIsAcceptable(const PlyReader& reader) override;

	void endParseFile() override;

	void* beginVertices(const std::string& elementName, size_t vertexCount) override;

	void processVertex(const std::string& elementName) override;

	void endVertices(const std::string& elementName) override;

	void processFemElement(const std::string& elementName) override;

	/// Callback function, begin the processing of radius.
	/// \param elementName Name of the element.
	/// \param radiusCount Number of radii.
	/// \return memory for radius data to the reader.
	void* beginRadius(const std::string& elementName, size_t radiusCount);

	/// Callback function, end the processing of radius.
	/// \param elementName Name of the element.
	void endRadius(const std::string& elementName);

	void processBoundaryCondition(const std::string& elementName) override;

private:
	/// Vertex data containing 6 dofs (3 translational and 3 rotational)
	struct Vertex6DData
	{
		double x;
		double y;
		double z;
		int64_t overrun1; ///< Used to check for buffer overruns
		double thetaX;
		double thetaY;
		double thetaZ;
		int64_t overrun2; ///< Used to check for buffer overruns
	} m_vertexData;

	/// Flag to notify if the ply file provides rotational data for the vertices or not
	bool m_hasRotationDOF;

	/// Element's radius information
	double m_radius;
	/// Element's shear information
	bool m_enableShear;

	/// Fem1D mesh asset to contain the ply file information
	std::shared_ptr<Fem1D> m_mesh;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEM1DPLYREADERDELEGATE_H
