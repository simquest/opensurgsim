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

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESHUTILITIES_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESHUTILITIES_H

#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"

namespace SurgSim
{
namespace DataStructures
{

/// Helper function to load a mesh from a given filename, does NOT do path resolution.
/// \throws SurgSim::Framework::AssertionFailure if the reader does not contain mesh information.
/// \param filename Path to the file that is to be read.
/// \return the filled mesh a filled mesh if the reading succeeds, nullptr otherwise
template <class M>
std::shared_ptr<M> loadTriangleMesh(const std::string& filename);

std::shared_ptr<TriangleMeshPlain> loadTriangleMesh(const std::string& filename);

}
}

#include "SurgSim/DataStructures/TriangleMeshUtilities-inl.h"

#endif
