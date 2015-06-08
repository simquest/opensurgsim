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

#include "SurgSim/Physics/Fem3D.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"

namespace SurgSim
{

namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Physics::Fem3D, Fem3D)

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

bool Fem3D::doLoad(const std::string& filePath)
{
	return loadFemFile<Fem3DPlyReaderDelegate, Fem3D>(filePath);
}

} // namespace Physics
} // namespace SurgSim
