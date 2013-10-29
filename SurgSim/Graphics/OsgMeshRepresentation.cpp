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
	SURGSIM_ASSERT(mesh != nullptr) << "Can't set a nullptr mesh";
	SURGSIM_ASSERT(mesh->isValid()) << "An invalid mesh was passed to OsgMeshRepresentation";

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

	SURGSIM_ASSERT(mesh->isValid()) << "Mesh is invalid, " <<
		"Cannot use and invalid mesh for creating a MeshRepresentation";

	m_geometry = new osg::Geometry();
	m_mesh = mesh;
	// Copy Vertices from mesh
	m_vertexCount = mesh->getNumVertices();
	m_vertices = new osg::Vec3Array(m_vertexCount);
	m_vertices->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->setVertexArray(m_vertices);
	updateVertices();

	// Check if the color on the vertex is set then assume perVertex colors
	// otherwise assign one color

	if (mesh->getVertex(0).data.color.hasValue())
	{
		m_colors = new osg::Vec4Array(m_vertexCount);
		m_colors->setDataVariance(osg::Object::DYNAMIC);
		m_geometry->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
		updateColors();
	}
	else
	{
		m_colors = new osg::Vec4Array(1);
		(*m_colors)[0]= osg::Vec4(1.0f,1.0f,1.0f,1.0f);
		m_geometry->setColorArray(m_colors, osg::Array::BIND_OVERALL);
	}

	// Check if the texture coordinates on vertex[0] is set if yes, assume
	// that there are textures
	if (mesh->getVertex(0).data.texture.hasValue())
	{
		m_textureCoordinates = new osg::Vec2Array(m_vertices->size());
		m_geometry->setTexCoordArray(0,m_textureCoordinates, osg::Array::BIND_PER_VERTEX);

		osg::Vec2 coordinates;
		osg::Vec2 defaultCoordinates(0.5f, 0.5f);
		for (size_t i=0; i<m_vertices->size(); ++i)
		{
			if (! m_mesh->getVertex(i).data.texture.hasValue())
			{
				SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
					"OsgMeshRepresentation::setMesh(): Missing texture coordinates for Vertex <" <<
					i << "> in Mesh, using default coordinates";
				(*m_textureCoordinates)[i] = defaultCoordinates;
			}
			(*m_textureCoordinates)[i] = SurgSim::Graphics::toOsg(mesh->getVertex(i).data.texture.getValue());
		}
	}

	// Copy triangles from mesh
	size_t triangleCount = mesh->getNumTriangles();
	unsigned int* triangles = new unsigned int[triangleCount*3];
	for (size_t i=0; i < triangleCount; ++i)
	{
		triangles[i*3] = mesh->getTriangle(i).verticesId[0];
		triangles[i*3+1] = mesh->getTriangle(i).verticesId[1];
		triangles[i*3+2] = mesh->getTriangle(i).verticesId[2];
	}

	m_set = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,triangleCount*3,triangles);

	m_geometry->addPrimitiveSet(m_set);

	// Create normals, currently per triangle
	m_normals = new osg::Vec3Array(m_vertices->size());
	m_normals->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->setNormalArray(m_normals,osg::Array::BIND_PER_VERTEX);
	updateNormals();

	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(m_geometry);

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

void OsgMeshRepresentation::update(double dt)
{
	// We probably should lock access to the mesh at this time ...
	SURGSIM_ASSERT(m_mesh->getNumVertices() == m_vertexCount) <<
		"Mesh structure changed, the OsgMeshRepresentation does not support this at the moment";

	updateVertices();

	// Only update colors if there is an actual color array
	if (m_colors->getDataVariance() == osg::Object::DYNAMIC)
	{
		updateColors();
	}
	updateNormals();

	m_set->dirty();
	m_geometry->dirtyDisplayList();

	// Rather than dirtying the bounds we should recalculate them on the fly
	m_geometry->dirtyBound();
}

void OsgMeshRepresentation::updateColors()
{
	static osg::Vec4 defaultColor(0.8f, 0.2f, 0.2f, 1.0f);

	for (size_t i=0; i<m_vertices->size(); ++i)
	{
		if (! m_mesh->getVertex(i).data.color.hasValue())
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics")) <<
				"OsgMeshRepresentation::setMesh(): Missing Vertex Color for Vertex <" <<
				i << "> in Mesh, using default";
			(*m_colors)[i] = defaultColor;
		}
		else
		{
			(*m_colors)[i] = SurgSim::Graphics::toOsg(m_mesh->getVertex(i).data.color.getValue());
		}
	}
	m_switch->dirtyBound();
}

void OsgMeshRepresentation::updateVertices()
{
	for (size_t i=0; i < m_vertexCount; ++i)
	{
		(*m_vertices)[i].set(toOsg(m_mesh->getVertexPosition(i)));
	}

	// Recalculate the bounding box here
}

void OsgMeshRepresentation::updateNormals()
{
	// Generate normals from geometry
	osg::TriangleIndexFunctor<TriangleNormalGenerator> normalGenerator;
	normalGenerator.set(m_vertices, m_normals);
	m_geometry->accept(normalGenerator);
	normalGenerator.normalize();
}


}; // Graphics

}; // SurgSim
