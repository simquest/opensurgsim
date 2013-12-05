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

#ifndef SURGSIM_TESTING_TESTCUBE_H
#define SURGSIM_TESTING_TESTCUBE_H

#include <vector>
#include <SurgSim/Math/Vector.h>

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

/// \file
/// Data for a cube, vertices, color, textures and triangles
/// This cube is for using as a mesh. OpenGl does not support per face attributes, therefore
/// to implement sharp edges vertices are duplicated over different faces, for a cube all
/// vertices need to exist as three copies

#include <memory>

namespace SurgSim
{

namespace Testing
{
namespace Cube
{

/// Fill our standard structures with the correct data types from the cube data
void makeCube(std::vector<Vector3d>* vertices,
	std::vector<Vector4d>* colors,
	std::vector<Vector2d>* textures,
	std::vector<unsigned int>* triangles);


}; // Cube
}; // Testing
}; // SurgSim

#endif