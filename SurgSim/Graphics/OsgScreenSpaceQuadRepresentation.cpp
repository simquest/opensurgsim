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

#include <memory>

#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgUniformBase.h"
#include "SurgSim/Graphics/Texture2d.h"
#include "SurgSim/Graphics/TextureRectangle.h"


#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Projection>
#include <osg/StateAttribute>
#include <osg/Switch>

namespace
{
enum TextureType
{
	TEXTURE_TYPE_RECTANGLE,
	TEXTURE_TYPE_POWER_OF_TWO
};
}

namespace SurgSim
{
namespace Graphics
{


OsgScreenSpaceQuadRepresentation::OsgScreenSpaceQuadRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	ScreenSpaceQuadRepresentation(name),
	m_scale(1.0, 1.0, 1.0)
{
	m_switch = new osg::Switch;
	m_switch->setName(name + " Switch");

	m_transform = new osg::PositionAttitudeTransform();
	m_transform->setName(name + " Transform");

	m_geode = new osg::Geode;

	// Make the quad
	float depth = 0.0;
	m_geometry = osg::createTexturedQuadGeometry(
					 osg::Vec3(0.0, 0.0, depth),
					 osg::Vec3(1.0, 0.0, depth),
					 osg::Vec3(0.0, 1.0, depth));

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	m_geometry->setColorArray(colors);
	m_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	m_geometry->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

	m_geode->addDrawable(m_geometry);

	m_transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	m_transform->setCullingActive(false);
	m_transform->addChild(m_geode);

	// By default use float texture coordinates, this makes this useable without a texture set
	setTextureCoordinates(0.0, 0.0, 1.0, 1.0);

	m_switch->addChild(m_materialProxy);
	m_materialProxy->addChild(m_transform);

	m_textureUniform = std::make_shared<OsgUniform<std::shared_ptr<OsgTexture2d>>>("texture");
	m_rectangleTextureUniform = std::make_shared<OsgUniform<std::shared_ptr<OsgTextureRectangle>>>("texture");

	removeGroupReference(Representation::DefaultGroupName);
	addGroupReference(Representation::DefaultHudGroupName);
}

OsgScreenSpaceQuadRepresentation::~OsgScreenSpaceQuadRepresentation()
{

}

void OsgScreenSpaceQuadRepresentation::setSize(double width, double height)
{
	m_scale.x() = width;
	m_scale.y() = height;
	m_transform->setScale(m_scale);
}

void OsgScreenSpaceQuadRepresentation::getSize(double* width, double* height) const
{
	SURGSIM_ASSERT(width != nullptr && height  != nullptr) << "Cannot use a nullptr as an output parameter";
	*width = m_scale.x();
	*height = m_scale.y();
}

bool OsgScreenSpaceQuadRepresentation::setTexture(std::shared_ptr<Texture> texture)
{
	SURGSIM_ASSERT(texture != nullptr) << "Null texture passed to setTexture";
	std::shared_ptr<OsgTexture2d> osgTexture2d = std::dynamic_pointer_cast<OsgTexture2d>(texture);
	if (osgTexture2d != nullptr)
	{
		return setTexture(osgTexture2d);
	}

	std::shared_ptr<OsgTextureRectangle> osgTextureRectangle = std::dynamic_pointer_cast<OsgTextureRectangle>(texture);
	if (osgTextureRectangle != nullptr)
	{
		return setTexture(osgTextureRectangle);
	}

	return false;

}

bool OsgScreenSpaceQuadRepresentation::setTexture(std::shared_ptr<OsgTexture2d> osgTexture)
{
	m_textureUniform->set(osgTexture);

	if (m_texureType.hasValue() && m_texureType.getValue() == TEXTURE_TYPE_RECTANGLE)
	{
		SURGSIM_ASSERT(!isInitialized()) << "Cannot change the type of texture once the quad has been initialized.";
		m_rectangleTextureUniform->removeFromStateSet(m_switch->getOrCreateStateSet());
	}
	else
	{
		m_textureUniform->addToStateSet(m_switch->getOrCreateStateSet());
		m_texureType.setValue(TEXTURE_TYPE_POWER_OF_TWO);
	}

	setTextureCoordinates(0.0, 0.0, 1.0, 1.0);

	return true;
}

bool OsgScreenSpaceQuadRepresentation::setTexture(std::shared_ptr<OsgTextureRectangle> osgTexture)
{
	m_rectangleTextureUniform->set(osgTexture);

	if (m_texureType.hasValue() && m_texureType.getValue() == TEXTURE_TYPE_POWER_OF_TWO)
	{
		SURGSIM_ASSERT(!isInitialized()) << "Cannot change the type of texture once the quad has been initialized.";
		m_textureUniform->removeFromStateSet(m_switch->getOrCreateStateSet());
	}
	else
	{
		m_rectangleTextureUniform->addToStateSet(m_switch->getOrCreateStateSet());
		m_texureType.setValue(TEXTURE_TYPE_RECTANGLE);
	}

	int width, height;
	osgTexture->getSize(&width, &height);
	setTextureCoordinates(0.0, 0.0, static_cast<float>(width), static_cast<float>(height));

	return true;
}

void OsgScreenSpaceQuadRepresentation::setTextureCoordinates(float left, float bottom, float right, float top)
{
	osg::Vec2Array* tcoords = new osg::Vec2Array(4);
	(*tcoords)[0].set(left, top);
	(*tcoords)[1].set(left, bottom);
	(*tcoords)[2].set(right, bottom);
	(*tcoords)[3].set(right, top);
	m_geometry->setTexCoordArray(0, tcoords);
}

void OsgScreenSpaceQuadRepresentation::setLocation(double x, double y)
{
	SurgSim::Math::RigidTransform3d transform =
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), SurgSim::Math::Vector3d(x, y, 0));
	setLocalPose(transform);
}

void OsgScreenSpaceQuadRepresentation::getLocation(double* x, double* y)
{
	SURGSIM_ASSERT(x !=  nullptr && y != nullptr) << "Cannot use a nullptr as an output parameter.";
	SurgSim::Math::Vector3d position = getLocalPose().translation();

	*x = position.x();
	*y = position.y();
}


void OsgScreenSpaceQuadRepresentation::doUpdate(double dt)
{
	m_transform->setAttitude(osg::Quat(0.0, 0.0, 0.0, 1.0));
}


bool OsgScreenSpaceQuadRepresentation::doInitialize()
{
	bool result = true;

	// if the material was preassigned, don't create a default one
	if (getMaterial() == nullptr && m_texureType.hasValue())
	{
		result = false;
		std::shared_ptr<OsgMaterial> material;
		switch (m_texureType.getValue())
		{
			case TEXTURE_TYPE_POWER_OF_TWO:
				material = buildMaterial("Shaders/unlit_texture.vert", "Shaders/unlit_texture.frag");
				break;
			case TEXTURE_TYPE_RECTANGLE:
				material = buildMaterial("Shaders/unlit_texture.vert", "Shaders/unlit_texture_rectangle.frag");
				break;
			default:
				break;
		}


		if (material != nullptr)
		{
			setMaterial(material);
			result = true;
		}
	}

	return result;
}

}; // Graphics
}; // SurgSim
