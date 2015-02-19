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

#include "SurgSim/Graphics/OsgMeshRepresentation.h"

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/Vec3f>
#include <osgUtil/SmoothingVisitor>

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/TriangleNormalGenerator.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgMeshRepresentation, OsgMeshRepresentation);

OsgMeshRepresentation::OsgMeshRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	MeshRepresentation(name),
	m_updateOptions(UPDATE_OPTION_VERTICES),
	m_mesh(std::make_shared<Mesh>())
{
	// The actual size of the mesh is not known at this time, just allocate the
	// osg structures that are needed and add them to the geometry, and the node
	m_geometry = new osg::Geometry();

	// Set up vertices array
	m_vertices = new osg::Vec3Array();
	m_vertices->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->setVertexArray(m_vertices);
	m_geometry->setUseDisplayList(false);

	// Set up color array with default color
	m_colors = new osg::Vec4Array(1);
	(*m_colors)[0] = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
	m_geometry->setColorArray(m_colors, osg::Array::BIND_OVERALL);

	// Set up textureCoordinates array, texture coords are optional, don't add them to the
	// geometry yet
	m_textureCoordinates = new osg::Vec2Array(0);
	m_textureCoordinates->setDataVariance(osg::Object::DYNAMIC);

	// Set up primitive set for triangles
	m_triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);
	m_triangles->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->addPrimitiveSet(m_triangles);

	// Create normals, currently per triangle
	m_normals = new osg::Vec3Array();
	m_normals->setDataVariance(osg::Object::DYNAMIC);
	m_geometry->setNormalArray(m_normals, osg::Array::BIND_PER_VERTEX);

	m_geode = new osg::Geode();
	m_geode->addDrawable(m_geometry);
}

OsgMeshRepresentation::~OsgMeshRepresentation()
{
}

void OsgMeshRepresentation::loadMesh(const std::string& fileName)
{
	auto mesh = std::make_shared<Mesh>();
	mesh->load(fileName);
	setMesh(mesh);
}

void OsgMeshRepresentation::setMesh(std::shared_ptr<SurgSim::Framework::Asset> mesh)
{
	auto graphicsMesh = std::dynamic_pointer_cast<Mesh>(mesh);
	SURGSIM_ASSERT(graphicsMesh != nullptr) << "Mesh for OsgMeshRepresentation needs to be a SurgSim::Graphics::Mesh";
	m_mesh = graphicsMesh;
}

std::shared_ptr<Mesh> OsgMeshRepresentation::getMesh()
{
	return m_mesh;
}

void OsgMeshRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_ASSERT(shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH)
		<< "Shape for OsgMeshRepresentation needs to be a SurgSim::Math::MeshShape";
	auto meshShape = std::static_pointer_cast<SurgSim::Math::MeshShape>(shape);
	m_mesh = std::make_shared<Mesh>(*meshShape);
}

void OsgMeshRepresentation::doUpdate(double dt)
{
	SURGSIM_ASSERT(m_mesh->isValid()) << "The mesh in the OsgMeshRepresentation " << getName() << " is invalid.";

	if (m_mesh->getNumVertices() > 0)
	{
		if (m_geode->getNumParents() <= 0)
		{
			m_transform->addChild(m_geode);
		}

		int updateOptions = updateOsgArrays();
		updateOptions |= m_updateOptions;

		if ((updateOptions & (UPDATE_OPTION_VERTICES | UPDATE_OPTION_TEXTURES | UPDATE_OPTION_COLORS)) != 0)
		{
			updateVertices(updateOptions);
			m_geometry->dirtyDisplayList();
			m_geometry->dirtyBound();
		}

		if ((updateOptions & UPDATE_OPTION_TRIANGLES) != 0)
		{
			updateTriangles();
			m_triangles->dirty();
		}
	}
	else
	{
		if (m_geode->getNumParents() > 0)
		{
			m_transform->removeChild(m_geode);
		}
	}
}

bool OsgMeshRepresentation::doInitialize()
{
	return true;
}

void OsgMeshRepresentation::updateVertices(int updateOptions)
{
	static osg::Vec4d defaultColor(0.8, 0.2, 0.2, 1.0);
	static osg::Vec2d defaultTextureCoord(0.0, 0.0);

	bool updateColors = (updateOptions & UPDATE_OPTION_COLORS) != 0;
	bool updateTextures = (updateOptions & UPDATE_OPTION_TEXTURES) != 0;
	bool updateVertices = (updateOptions & UPDATE_OPTION_VERTICES) != 0;
	size_t vertexCount = m_mesh->getNumVertices();

	for (size_t i = 0; i < vertexCount; ++i)
	{
		Mesh::VertexType vertex = m_mesh->getVertex(i);
		if (updateVertices)
		{
			(*m_vertices)[i].set(toOsg(vertex.position));
		}
		if (updateColors)
		{
			(*m_colors)[i] = (vertex.data.color.hasValue()) ? toOsg(vertex.data.color.getValue()) : defaultColor;
		}
		if (updateTextures)
		{
			(*m_textureCoordinates)[i] =
				(vertex.data.texture.hasValue()) ? toOsg(vertex.data.texture.getValue()) : defaultTextureCoord;
		}
	}

	if (updateVertices)
	{
		updateNormals();
	}
}

void OsgMeshRepresentation::updateNormals()
{
	// Generate normals from geometry
	auto normalGenerator = createNormalGenerator(m_vertices, m_normals);
	m_geometry->accept(normalGenerator);
	normalGenerator.normalize();
}

void OsgMeshRepresentation::updateTriangles()
{
	int i = 0;
	m_triangles->resize(m_mesh->getNumTriangles() * 3);
	for (auto const& triangle : m_mesh->getTriangles())
	{
		if (triangle.isValid)
		{
			(*m_triangles)[i++] = triangle.verticesId[0];
			(*m_triangles)[i++] = triangle.verticesId[1];
			(*m_triangles)[i++] = triangle.verticesId[2];
		}
	}
}

int OsgMeshRepresentation::updateOsgArrays()
{
	int result = 0;

	size_t numVertices = m_mesh->getNumVertices();

	if (numVertices > m_vertices->size())
	{
		m_vertices->resize(numVertices);
		m_normals->resize(numVertices);

		m_vertices->setDataVariance(getDataVariance(UPDATE_OPTION_VERTICES));
		m_normals->setDataVariance(getDataVariance(UPDATE_OPTION_VERTICES));

		result |= UPDATE_OPTION_VERTICES;
	}

	// The first vertex determines what values the mesh should have ...
	Mesh::VertexType vertex = m_mesh->getVertex(0);

	if (vertex.data.color.hasValue() && numVertices > m_colors->size())
	{
		if (m_colors->size() > 1)
		{
			m_colors->setDataVariance(getDataVariance(UPDATE_OPTION_COLORS));
			m_geometry->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
		}
		m_colors->resize(numVertices);
		result |= UPDATE_OPTION_COLORS;
	}

	if (vertex.data.texture.hasValue() && numVertices > m_textureCoordinates->size())
	{
		bool setTextureArray = m_textureCoordinates->size() == 0;
		m_textureCoordinates->resize(numVertices);
		if (setTextureArray)
		{
			m_geometry->setTexCoordArray(0, m_textureCoordinates, osg::Array::BIND_PER_VERTEX);
			m_textureCoordinates->setDataVariance(getDataVariance(UPDATE_OPTION_TEXTURES));
		}
		result |= UPDATE_OPTION_TEXTURES;
	}

	if (m_mesh->getNumTriangles() * 3 > m_triangles->size())
	{
		m_triangles->resize(m_mesh->getNumTriangles() * 3);
		m_triangles->setDataVariance(getDataVariance(UPDATE_OPTION_TRIANGLES));
		result |= UPDATE_OPTION_TRIANGLES;
	}
	return result;
}

void OsgMeshRepresentation::setUpdateOptions(int val)
{
	if (val <= UPDATE_OPTION_ALL && val >= UPDATE_OPTION_NONE)
	{
		m_updateOptions = val;
	}
}

int OsgMeshRepresentation::getUpdateOptions() const
{
	return m_updateOptions;
}

osg::ref_ptr<osg::Geometry> OsgMeshRepresentation::getOsgGeometry()
{
	return m_geometry;
}

osg::Object::DataVariance OsgMeshRepresentation::getDataVariance(int updateOption)
{
	return ((m_updateOptions & updateOption) != 0) ? osg::Object::DYNAMIC : osg::Object::STATIC;
}

}; // Graphics
}; // SurgSim
