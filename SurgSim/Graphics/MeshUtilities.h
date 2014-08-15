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

#ifndef SURGSIM_GRAPHICS_MESHUTILITIES_H
#define SURGSIM_GRAPHICS_MESHUTILITIES_H

#include "SurgSim/DataStructures/TriangleMeshUtilities.h"
#include "SurgSim/Graphics/MeshPlyReaderDelegate.h"

namespace SurgSim
{
namespace DataStructures
{

/// Specialization for Graphics::Mesh
/// Helper function to load a mesh from a given filename, does NOT do path resolution.
/// \throws SurgSim::Framework::AssertionFailure if the reader does not contain mesh information.
/// \param filename Path to the file that is to be read.
/// \return the filled mesh a filled mesh if the reading succeeds, nullptr otherwise
template <>
std::shared_ptr<SurgSim::Graphics::Mesh> loadTriangleMesh(const std::string& fileName);

}
}


#endif