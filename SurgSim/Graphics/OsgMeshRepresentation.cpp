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
#include <osg/Switch>
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
	m_updateCount(0)
{
	m_meshSwitch = new osg::Switch();
	m_transform->addChild(m_meshSwitch);

	setMesh(std::make_shared<Mesh>());
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
	buildGeometry();
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
	size_t count = 0;

	SURGSIM_ASSERT(m_mesh->isValid()) << "The mesh in the OsgMeshRepresentation " << getName() << " is invalid.";

	// Early if there are no vertices
	if (m_mesh->getNumVertices() == 0)
	{
		m_meshSwitch->setAllChildrenOff();
		return;
	}

	m_meshSwitch->setSingleChildOn(0);

	// Early exit if the mesh was not updated, no need to update the osg data
	size_t updateCount = m_mesh->getUpdateCount();
	if (m_updateCount == updateCount)
	{
		return;
	}
	m_updateCount = updateCount;

	int updateOptions = updateOsgArrays(m_geometry);
	updateOptions |= m_updateOptions;

	if ((updateOptions & UPDATE_OPTION_TRIANGLES) != 0)
	{
		updateTriangles(m_geometry);
	}

	if ((updateOptions & (UPDATE_OPTION_VERTICES | UPDATE_OPTION_TEXTURES | UPDATE_OPTION_COLORS)) != 0)
	{
		updateVertices(m_geometry, updateOptions);
		updateTangents();
		m_geometry->dirtyDisplayList();
		m_geometry->dirtyBound();
		m_geometry->getBound();
	}

	++count;
}

bool OsgMeshRepresentation::doInitialize()
{
	return true;
}

void OsgMeshRepresentation::updateVertices(osg::Geometry* geometry, int updateOptions)
{
	static osg::Vec4d defaultColor(0.8, 0.2, 0.2, 1.0);
	static osg::Vec2d defaultTextureCoord(0.0, 0.0);

	bool updateColors = (updateOptions & UPDATE_OPTION_COLORS) != 0;
	bool updateTextures = (updateOptions & UPDATE_OPTION_TEXTURES) != 0;
	bool updateVertices = (updateOptions & UPDATE_OPTION_VERTICES) != 0;

	auto vertices = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
	auto colors = static_cast<osg::Vec4Array*>(geometry->getColorArray());
	auto textureCoords = static_cast<osg::Vec2Array*>(geometry->getTexCoordArray(0));

	size_t index = 0;
	for (const auto& vertex : m_mesh->getVertices())
	{
		if (updateVertices)
		{
			(*vertices)[index].set(toOsg(vertex.position));
		}
		if (updateColors)
		{
			(*colors)[index] = (vertex.data.color.hasValue()) ? toOsg(vertex.data.color.getValue()) : defaultColor;
		}
		if (updateTextures)
		{
			(*textureCoords)[index] =
				(vertex.data.texture.hasValue()) ? toOsg(vertex.data.texture.getValue()) : defaultTextureCoord;
		}
		++index;
	}

	if (updateVertices)
	{
		updateNormals(geometry);
	}
	vertices->dirty();
}

void OsgMeshRepresentation::updateNormals(osg::Geometry* geometry)
{
	// Generate normals from geometry
	auto vertices = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
	auto normals = static_cast<osg::Vec3Array*>(geometry->getNormalArray());
	auto normalGenerator = createNormalGenerator(vertices, normals);
	geometry->accept(normalGenerator);
	normalGenerator.normalize();
	normals->dirty();
}

void OsgMeshRepresentation::updateTriangles(osg::Geometry* geometry)
{
	osg::Geometry::DrawElementsList drawElements;
	geometry->getDrawElementsList(drawElements);
	auto triangles = static_cast<osg::DrawElementsUInt*>(drawElements[0]);

	size_t i = 0;
	for (auto const& triangle : m_mesh->getTriangles())
	{
		if (triangle.isValid)
		{
			(*triangles)[i++] = triangle.verticesId[0];
			(*triangles)[i++] = triangle.verticesId[1];
			(*triangles)[i++] = triangle.verticesId[2];
		}
	}
	triangles->dirty();
}

int OsgMeshRepresentation::updateOsgArrays(osg::Geometry* geometry)
{
	int result = 0;

	size_t numVertices = m_mesh->getNumVertices();

	auto vertices = static_cast<osg::Vec3Array*>(geometry->getVertexArray());
	auto normals = static_cast<osg::Vec3Array*>(geometry->getNormalArray());
	if (numVertices != vertices->size())
	{
		vertices->resize(numVertices);
		normals->resize(numVertices);

		result |= UPDATE_OPTION_VERTICES;
	}
	vertices->setDataVariance(getDataVariance(UPDATE_OPTION_VERTICES));
	normals->setDataVariance(getDataVariance(UPDATE_OPTION_VERTICES));

	// The first vertex determines what values the mesh should have ...
	Mesh::VertexType vertex = m_mesh->getVertex(0);

	auto colors = static_cast<osg::Vec4Array*>(geometry->getColorArray());
	if (vertex.data.color.hasValue() && numVertices > colors->size())
	{
		if (colors->size() > 1)
		{
			geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
		}
		colors->resize(numVertices);
		result |= UPDATE_OPTION_COLORS;
	}
	colors->setDataVariance(getDataVariance(UPDATE_OPTION_COLORS));

	auto textureCoords = static_cast<osg::Vec2Array*>(geometry->getTexCoordArray(0));
	if (vertex.data.texture.hasValue())
	{
		if (textureCoords == nullptr)
		{
			textureCoords = new osg::Vec2Array(0);
			geometry->setTexCoordArray(0, textureCoords, osg::Array::BIND_PER_VERTEX);
		}
		textureCoords->resize(numVertices);
		result |= UPDATE_OPTION_TEXTURES;
	}
	if (textureCoords != nullptr)
	{
		textureCoords->setDataVariance(getDataVariance(UPDATE_OPTION_TEXTURES));
	}

	osg::Geometry::DrawElementsList drawElements;
	geometry->getDrawElementsList(drawElements);
	auto triangles = static_cast<osg::DrawElementsUInt*>(drawElements[0]);

	if (m_mesh->getNumTriangles() * 3 != triangles->size())
	{
		triangles->resize(m_mesh->getNumTriangles() * 3);
		result |= UPDATE_OPTION_TRIANGLES;
	}
	triangles->setDataVariance(getDataVariance(UPDATE_OPTION_TRIANGLES));

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

osg::ref_ptr<osg::Geometry> OsgMeshRepresentation::getOsgGeometry() const
{
	return m_geometry;
}

osg::Object::DataVariance OsgMeshRepresentation::getDataVariance(int updateOption)
{
	return ((m_updateOptions & updateOption) != 0) ? osg::Object::DYNAMIC : osg::Object::STATIC;
}

void OsgMeshRepresentation::buildGeometry()
{
	// Remove old Geometry nodes
	m_meshSwitch->removeChildren(0, m_meshSwitch->getNumChildren());

	m_geometry = new osg::Geometry;

	m_geometry->setUseDisplayList(false);

	// Create Standard arrays zero size, updateOsgArrays will take care of the correct sizing and
	// setting the correct data variance on them

	// Set up vertices array
	auto vertices = new osg::Vec3Array();
	m_geometry->setVertexArray(vertices);

	// Create normals
	auto normals = new osg::Vec3Array();
	m_geometry->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);

	// Set up color array with default color
	auto colors = new osg::Vec4Array(1);
	(*colors)[0] = osg::Vec4(0.8f, 0.8f, 1.0f, 1.0f);
	m_geometry->setColorArray(colors, osg::Array::BIND_OVERALL);

	// Set up primitive set for triangles
	auto triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);
	m_geometry->addPrimitiveSet(triangles);

	auto geode = new osg::Geode;
	geode->addDrawable(m_geometry);
	m_meshSwitch->addChild(geode);
}

}; // Graphics
}; // SurgSim
