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

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Physics/FemElementMesh.h"
#include "SurgSim/Physics/FemElement1DMeshPlyReaderDelegate.h"

template<>
std::string SurgSim::DataStructures::TriangleMesh<SurgSim::Physics::FemElementStructs::RotationVectorData,
	EmptyData, EmptyData>::m_className = "SurgSim::Physics::FemElementMesh";

namespace SurgSim
{

namespace Physics
{

FemElement1DMesh::FemElement1DMesh()
{
}

void FemElement1DMesh::load(const std::string& fileName)
{
	doLoad(fileName);
}

bool FemElement1DMesh::isEnableShear() const
{
	return m_enableShear;
}

double FemElement1DMesh::getRadius() const
{
	return m_radius;
}

void FemElement1DMesh::setEnableShear(bool enableShear)
{
	m_enableShear = enableShear;
}

void FemElement1DMesh::setRadius(double radius)
{
	m_radius = radius;
}

bool FemElement1DMesh::doLoad(const std::string& filePath)
{
	SurgSim::DataStructures::PlyReader reader(filePath);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filePath << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<FemElement1DMeshPlyReaderDelegate>(std::dynamic_pointer_cast<FemElement1DMesh>(shared_from_this()));
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << filePath << "' does not have the property required by FEM element mesh.";
		return false;
	}

	return true;
}

} // namespace Physics
} // namespace SurgSim
