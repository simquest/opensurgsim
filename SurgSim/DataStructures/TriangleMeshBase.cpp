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

#include "SurgSim/DataStructures/TriangleMeshBase.h"

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"

namespace SurgSim
{
namespace DataStructures
{

std::shared_ptr<TriangleMeshBase<EmptyData, EmptyData, EmptyData>> loadTriangleMesh(const std::string& fileName)
{
	std::shared_ptr<TriangleMeshPlyReaderDelegate> triangleMeshDelegate
		= std::make_shared<TriangleMeshPlyReaderDelegate>();

	PlyReader reader(fileName);
	if (reader.isValid())
	{
		SURGSIM_ASSERT(reader.setDelegate(triangleMeshDelegate)) <<
			"The input file " << fileName << " does not have the property required by triangle mesh.";
		reader.parseFile();
	}

	return triangleMeshDelegate->getMesh();
}

};  // namespace DataStructures
};  // namespace SurgSim

