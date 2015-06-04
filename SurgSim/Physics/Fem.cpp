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
#include "SurgSim/Physics/Fem.h"
#include "SurgSim/Physics/Fem1DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem2DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"

template<>
std::string SurgSim::DataStructures::TriangleMesh<SurgSim::Physics::FemElementStructs::Fem1DVectorData,
	EmptyData, EmptyData>::m_className = "SurgSim::Physics::Fem1D";

template<>
std::string SurgSim::DataStructures::TriangleMesh<SurgSim::Physics::FemElementStructs::Fem2DVectorData,
	EmptyData, EmptyData>::m_className = "SurgSim::Physics::Fem2D";

template<>
std::string SurgSim::DataStructures::TriangleMesh<SurgSim::Physics::FemElementStructs::Fem3DVectorData,
	EmptyData, EmptyData>::m_className = "SurgSim::Physics::Fem3D";

namespace SurgSim
{

namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Physics::Fem1D, Fem1D)

Fem1D::Fem1D() : Fem()
{
}

bool Fem1D::doLoad(const std::string& filePath)
{
	SurgSim::DataStructures::PlyReader reader(filePath);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filePath << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<Fem1DPlyReaderDelegate>(
						std::dynamic_pointer_cast<Fem1D>(shared_from_this()));
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << filePath << "' does not have the property required by FEM element mesh.";
		return false;
	}

	return true;
}

Fem2D::Fem2D() : Fem()
{
}

bool Fem2D::doLoad(const std::string& filePath)
{
	SurgSim::DataStructures::PlyReader reader(filePath);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filePath << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<Fem2DPlyReaderDelegate>(
						std::dynamic_pointer_cast<Fem2D>(shared_from_this()));
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << filePath << "' does not have the property required by FEM element mesh.";
		return false;
	}

	return true;
}

Fem3D::Fem3D()
{
}

size_t Fem3D::addCube(std::shared_ptr<CubeType> cube)
{
	m_cubeElements.push_back(cube);
	return m_cubeElements.size() - 1;
}

size_t Fem3D::getNumCubes() const
{
	return m_cubeElements.size();
}

const std::vector<std::shared_ptr<CubeType>>&Fem3D::getCubes() const
{
	return m_cubeElements;
}

std::vector<std::shared_ptr<CubeType> >& Fem3D::getCubes()
{
	return m_cubeElements;
}

std::shared_ptr<CubeType> Fem3D::getCube(size_t id) const
{
	return m_cubeElements[id];
}

void Fem3D::removeCube(size_t id)
{
	m_cubeElements.erase(m_cubeElements.begin() + id);
}

bool Fem3D::doLoad(const std::string& filePath)
{
	SurgSim::DataStructures::PlyReader reader(filePath);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filePath << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<Fem3DPlyReaderDelegate>(
						std::dynamic_pointer_cast<Fem3D>(shared_from_this()));
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
