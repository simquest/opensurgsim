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
#include <osg/TriangleIndexFunctor>

namespace SurgSim
{
namespace Graphics
{


/// Triangle index functor which calculates normals for the vertices of a geometry, use 
/// createNormalGenerator to instantiate this
class TriangleNormalGenerator
{
public:

	/// Sets the arrays required to generate normals
	/// \pre vertexArray and normalArray, need to have the same number of entries and not be nullptr
	/// \param vertexArray Array containing vertex positions
	/// \param normalArray Array to store calculated normals
	void set(osg::Vec3Array* vertexArray,
			 osg::Vec3Array* normalArray);

	/// Normalizes the calculated normals, this needs to be called after the pass to normalize all the normals
	/// Due to the osg way this object is called there is no real good way of having this called automatically
	void normalize();

	/// Resets all calculated normals to 0.
	void reset();

	/// Calculates the triangle normal and adds it to each adjacent vertex normal.
	/// \param vertexIndex1 First triangle vertex index
	/// \param vertexIndex2 Second triangle vertex index
	/// \param vertexIndex3 Third triangle vertex index
	void operator() (unsigned int vertexIndex1, unsigned int vertexIndex2, unsigned int vertexIndex3);

protected:
	/// Constructor
	TriangleNormalGenerator();

private:

	/// Array containing vertex positions
	osg::ref_ptr<osg::Vec3Array> m_vertexArray;

	/// Array storing calculated normals
	osg::ref_ptr<osg::Vec3Array> m_normalArray;

	/// Size of vertex and normal array
	size_t m_size;
};

osg::TriangleIndexFunctor<TriangleNormalGenerator> createNormalGenerator(osg::Vec3Array* vertexArray, osg::Vec3Array* normalArray);


}; // Graphics
}; // SurgSim

#endif