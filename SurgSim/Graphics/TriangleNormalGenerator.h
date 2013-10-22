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

#ifndef SURGSIM_GRAPHICS_TRIANGLENORMALGENERATOR_H
#define SURGSIM_GRAPHICS_TRIANGLENORMALGENERATOR_H

#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>

namespace SurgSim
{
namespace Graphics
{


/// Triangle index functor which calculates normals for the vertices of a geometry
class TriangleNormalGenerator
{
public:
	/// Constructor
	TriangleNormalGenerator();

	/// Sets the arrays required to generate normals
	/// \param vertexArray Array containing vertex positions
	/// \param normalArray Array to store calculated normals
	void set(const osg::Vec3Array* vertexArray,
			 osg::Vec3Array* normalArray);

	/// Normalizes the calculated normals, this needs to be called after the pass to normalize all the normals
	void normalize();

	/// Resets all calculated normals to 0.
	void reset();

	/// Calculates the triangle normal and adds it to each adjacent vertex normal.
	/// \param vertexIndex1 First triangle vertex index
	/// \param vertexIndex2 Second triangle vertex index
	/// \param vertexIndex3 Third triangle vertex index
	void operator() (unsigned int vertexIndex1, unsigned int vertexIndex2, unsigned int vertexIndex3);


private:
	/// Array containing vertex positions
	const osg::Vec3Array* m_vertexArray;

	/// Array storing calculated normals
	osg::Vec3Array* m_normalArray;
};


}; // Graphics
}; // SurgSim

#endif