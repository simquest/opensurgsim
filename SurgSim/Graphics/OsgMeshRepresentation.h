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

#ifndef SURGSIM_GRAPHICS_OSGMESHREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGMESHREPRESENTATION_H

#include <memory>

#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/MeshRepresentation.h>
#include <SurgSim/Graphics/Mesh.h>

#include <osg/ref_ptr>
#include <osg/Array>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
	class Geometry;
}

namespace SurgSim
{
namespace Graphics
{

class OsgTexture;
class Texture;


/// Implementation of a MeshRepresentation for rendering under osg.
class OsgMeshRepresentation : public OsgRepresentation, public MeshRepresentation
{
public:

	/// Constructor
	explicit OsgMeshRepresentation(const std::string& name);
	~OsgMeshRepresentation();

	/// Sets the mesh.
	/// \param	mesh	The mesh.
	/// \return	true if it succeeds, false if it fails.
	virtual bool setMesh(std::shared_ptr<Mesh> mesh) override;

	void updateVertices();


	/// Gets the mesh.
	/// \return	The mesh.
	virtual std::shared_ptr<Mesh> getMesh() override;

	/// Sets the mesh to render as a wire frame.
	/// \param	val	true to value.
	void setDrawAsWireFrame(bool val);

	/// Updates the mesh with the new vertex positions.
	/// \param	dt	The dt.
	virtual void update(double dt);

private:

	/// The mesh.
	std::shared_ptr<Mesh> m_mesh;

	
	/// The Osg Geometry.
	osg::ref_ptr<osg::Geometry> m_geometry;	

	size_t m_vertexCount;

	osg::ref_ptr<osg::Vec3Array> m_vertices;
	osg::ref_ptr<osg::Vec4Array> m_colors;
	osg::ref_ptr<osg::Vec3Array> m_normals;
	osg::ref_ptr<osg::Vec2Array> m_textureCoordinates;

	void updateColors();
	void updateNormals();

};


#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#endif