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

#include <SurgSim/Graphics/OsgMeshRepresentation.h>

#include <SurgSim/Graphics/OsgConversions.h>
#include <SurgSim/Graphics/OsgTexture.h>
#include <SurgSim/Graphics/Texture.h>
#include <SurgSim/Graphics/TriangleNormalGenerator.h>

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Switch>
#include <osg/Vec3f>
#include <osg/PolygonMode>
#include <osg/TriangleIndexFunctor>
#include <osg/PositionAttitudeTransform>

#include <osgUtil/SmoothingVisitor>


namespace SurgSim
{
namespace Graphics
{

OsgMeshRepresentation::OsgMeshRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	MeshRepresentation(name)
{

}

OsgMeshRepresentation::~OsgMeshRepresentation()
{

}

bool OsgMeshRepresentation::setMesh(std::shared_ptr<Mesh> mesh)
{
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	// Copy Vertices from mesh
	size_t vertexCount = mesh->getNumVertices();
	osg::Vec3Array* vertices = new osg::Vec3Array(vertexCount);

	for (size_t i=0; i < vertexCount; ++i)
	{
		(*vertices)[i].set(toOsg(mesh->getVertexPosition(i)));
	}
	geometry->setVertexArray(vertices);

	// Check if the color on the vertex is set then assume perVertex colors
	// otherwise assign one color
	osg::Vec4Array* colors;
	if (mesh->getVertex(0).data.color.hasValue())
	{
		colors = new osg::Vec4Array(vertices->size());
		osg::Vec4 color;
		osg::Vec4 defaultColor(0.8f, 0.2f, 0.2f, 1.0f);
		for (size_t i=0; i<vertices->size(); ++i)
		{
			if (! mesh->getVertex(i).data.color.hasValue())
			{
				SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
					"OsgMeshRepresentation::setMesh(): Missing Vertex Color for Vertex <" <<
					i << "> in Mesh, using default";
			}
			(*colors)[i] = (mesh->getVertex(i).data.color.hasValue()) ?
				SurgSim::Graphics::toOsg(mesh->getVertex(i).data.color.getValue()) : defaultColor;
			geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
		}
	}
	else
	{
		colors = new osg::Vec4Array(1);
		(*colors)[0]= osg::Vec4(1.0f,1.0f,1.0f,1.0f);
		geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
	}

	// Check if the texture coordinates on vertex[0] is set if yes, assume
	// that there are textures
	osg::Vec2Array* textureCoords;
	if (mesh->getVertex(0).data.texture.hasValue())
	{
		textureCoords = new osg::Vec2Array(vertices->size());
		osg::Vec2 coordinates;
		osg::Vec2 defaultCoordinates(0.5f, 0.5f);
		for (size_t i=0; i<vertices->size(); ++i)
		{
			if (! mesh->getVertex(i).data.texture.hasValue())
			{
				SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
					"OsgMeshRepresentation::setMesh(): Missing texture coordinates for Vertex <" <<
					i << "> in Mesh, using default coordinates";
			}
			(*textureCoords)[i] = (mesh->getVertex(i).data.texture.hasValue()) ?
				SurgSim::Graphics::toOsg(mesh->getVertex(i).data.texture.getValue()) : defaultCoordinates;
		}
		geometry->setTexCoordArray(0,textureCoords, osg::Array::BIND_PER_VERTEX);
	}
		
	// Copy triangles from mesh
	size_t triangleCount = mesh->getNumTriangles();
	unsigned int* triangles = new unsigned int[triangleCount*3];
	for (size_t i=0,j=0; i < triangleCount; ++i, j+=3)
	{
		triangles[j] = mesh->getTriangle(i).verticesId[0];
		triangles[j+1] = mesh->getTriangle(i).verticesId[1];
		triangles[j+2] = mesh->getTriangle(i).verticesId[2];
	}
	geometry->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,triangleCount*3,triangles));

	// Create normals, currently per triangle
	osg::Vec3Array* normals = new osg::Vec3Array(vertices->size());
	geometry->setNormalArray(normals,osg::Array::BIND_PER_VERTEX);

	// Generate normals from geometry
	osg::TriangleIndexFunctor<TriangleNormalGenerator> normalGenerator;
	normalGenerator.set(vertices, normals);
	geometry->accept(normalGenerator);
	normalGenerator.normalize();

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geometry);

	m_transform->addChild(geode);


	return true;
}

std::shared_ptr<Mesh> OsgMeshRepresentation::getMesh()
{
	return m_mesh;
}

void OsgMeshRepresentation::setDrawAsWireFrame(bool val)
{
	osg::StateSet* state = m_switch->getOrCreateStateSet();

	osg::ref_ptr<osg::PolygonMode> polygonMode;
	if (val)
	{
		 polygonMode = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);

	}
	else
	{
		polygonMode = new osg::PolygonMode(osg::PolygonMode::FRONT, osg::PolygonMode::FILL);
	}

	state->setAttributeAndModes(polygonMode, osg::StateAttribute::ON);
}

}; // Graphics

}; // SurgSim
