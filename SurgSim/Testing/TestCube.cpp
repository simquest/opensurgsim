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

#include <SurgSim/Testing/TestCube.h>

namespace SurgSim
{
namespace Testing
{
namespace Cube
{

int numVertices = 24;

double vertexData[] =
{
	-0.5,-0.5,-0.5,
	-0.5,-0.5, 0.5,
	0.5,-0.5, 0.5,
	0.5,-0.5,-0.5,
	-0.5, 0.5,-0.5,
	-0.5, 0.5, 0.5,
	0.5, 0.5, 0.5,
	0.5, 0.5,-0.5,

	-0.5,-0.5,-0.5,
	-0.5,-0.5, 0.5,
	0.5,-0.5, 0.5,
	0.5,-0.5,-0.5,
	-0.5, 0.5,-0.5,
	-0.5, 0.5, 0.5,
	0.5, 0.5, 0.5,
	0.5, 0.5,-0.5,

	-0.5,-0.5,-0.5,
	-0.5,-0.5, 0.5,
	0.5,-0.5, 0.5,
	0.5,-0.5,-0.5,
	-0.5, 0.5,-0.5,
	-0.5, 0.5, 0.5,
	0.5, 0.5, 0.5,
	0.5, 0.5,-0.5
};

double colorData[] =
{
	0.9,0.0,0.0,1.0,
	0.9,0.0,0.0,1.0,
	0.9,0.0,0.0,1.0,
	0.9,0.0,0.0,1.0,
	0.9,0.5,0.5,1.0,
	0.9,0.5,0.5,1.0,
	0.9,0.5,0.5,1.0,
	0.9,0.5,0.5,1.0,

	0.0,0.9,0.0,1.0,
	0.0,0.9,0.0,1.0,
	0.5,0.9,0.5,1.0,
	0.5,0.9,0.5,1.0,
	0.0,0.9,0.0,1.0,
	0.0,0.9,0.0,1.0,
	0.5,0.9,0.5,1.0,
	0.5,0.9,0.5,1.0,

	0.0,0.0,0.9,1.0,
	0.5,0.5,0.9,1.0,
	0.5,0.5,0.9,1.0,
	0.0,0.0,0.9,1.0,
	0.0,0.0,0.9,1.0,
	0.5,0.5,0.9,1.0,
	0.5,0.5,0.9,1.0,
	0.0,0.0,0.9,1.0
};

double textureData[] =
{
	0.25,0.25,
	0.25,0.50,
	0.0, 0.50,
	0.0, 0.25,
	0.50,0.25,
	0.50,0.50,
	0.50,0.50,
	0.50,0.25,

	0.25,0.25,
	0.25,0.50,
	1.0, 0.50,
	1.0, 0.25,
	0.50,0.25,
	0.50,0.50,
	0.75,0.50,
	0.75,0.25,

	0.50,0.0 ,
	0.50,0.75,
	0.75,0.75,
	0.75,0.0,
	0.50,0.25,
	0.50,0.50,
	0.75,0.50,
	0.75,0.25
};

unsigned int numTriangles = 12;
unsigned int triangleData[] =
{
	0, 3, 2,
	0, 2, 1,
	4, 6, 7,
	4, 5, 6,
	8, 9,13,
	8,13,12,
	10,15,14,
	10,11,15,
	16,20,23,
	16,23,19,
	17,18,22,
	17,22,21
};

void makeCube(std::vector<Vector3d>* vertices,
			std::vector<Vector4d>* colors,
			std::vector<Vector2d>* textures,
			std::vector<unsigned int>* triangles)
{
vertices->resize(numVertices);
colors->resize(numVertices);
textures->resize(numVertices);

double scale = 1.0;

for (int i=0; i<numVertices; ++i)
{
	(*vertices)[i] = Vector3d(vertexData[3*i]*scale, vertexData[3*i+1]*scale, vertexData[3*i+2]*scale);
	(*colors)[i] = Vector4d(colorData[4*i], colorData[4*i+1], colorData[4*i+2], colorData[4*i+3]);
	(*textures)[i] = Vector2d(textureData[2*i], textureData[2*i+1]);
}

triangles->resize(numTriangles*3);
std::copy(triangleData, triangleData+12*3,std::begin(*triangles));
}

}; // Cube
}; // Testing
}; // SurgSim
