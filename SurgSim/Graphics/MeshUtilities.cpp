// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "SurgSim/Graphics/MeshUtilities.h"

template <>
std::shared_ptr<SurgSim::Graphics::Mesh> SurgSim::DataStructures::loadTriangleMesh(const std::string& fileName)
{
	SurgSim::DataStructures::PlyReader reader(fileName);
	auto delegate = std::make_shared<SurgSim::Graphics::MeshPlyReaderDelegate>();
	SURGSIM_ASSERT(reader.setDelegate(delegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	return delegate->getMesh();
}

