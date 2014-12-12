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

#include "SurgSim/Graphics/OsgTextRepresentation.h"

#include <osg/Geode>
#include <osg/Drawable>
#include <osg/PositionAttitudeTransform>
#include <osgText/Text>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Graphics/OsgConversions.h"
#include "SurgSim/Graphics/OsgFont.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Graphics
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgTextRepresentation, OsgTextRepresentation);

OsgTextRepresentation::OsgTextRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	TextRepresentation(name),
	m_geode(new osg::Geode),
	m_textNode(new osgText::Text),
	m_needUpdate(true)
{
	m_textNode->setDataVariance(osg::Object::DYNAMIC);
	m_textNode->setUseDisplayList(false);
	m_characterSize = m_textNode->getCharacterHeight();
	m_geode->addDrawable(m_textNode);

	m_transform->addChild(m_geode);
	m_transform->setCullingActive(false);
	m_transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	// Try to find default font, if this fails OSG will fall back to its own default, only a warning warranted
	auto font = std::make_shared<OsgFont>();
	try
	{
		font->load("Fonts/Vera.ttf");
	}
	catch (std::exception e)
	{
		font = nullptr;
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics"))
				<< "Could not set default font Fonts/Vera.ttf";
	}

	m_font = font;

	// For now setup for HUD display
	m_textNode->setCharacterSizeMode(osgText::TextBase::SCREEN_COORDS);
	removeGroupReference(Representation::DefaultGroupName);
	addGroupReference(Representation::DefaultHudGroupName);
}

OsgTextRepresentation::~OsgTextRepresentation()
{
}

void OsgTextRepresentation::setLocation(double x, double y)
{
	setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(x, y, 0.0)));
}

void OsgTextRepresentation::getLocation(double* x, double* y) const
{
	SURGSIM_ASSERT(x !=  nullptr && y != nullptr) << "Cannot use a nullptr as an output parameter.";
	Vector3d position = getLocalPose().translation();

	*x = position.x();
	*y = position.y();
}

void OsgTextRepresentation::setText(const std::string& text)
{
	boost::mutex::scoped_lock lock(m_parameterMutex);
	m_text = text;
	m_needUpdate = true;
}

std::string OsgTextRepresentation::getText() const
{
	return m_text;
}

void OsgTextRepresentation::doUpdate(double dt)
{
	{
		boost::mutex::scoped_lock lock(m_parameterMutex);
		if (m_needUpdate)
		{
			// Osg will catch which ones have been updated, no need for more fine grained control.
			// Each of these may trigger an operation that rebuilds the internal structure of the text drawable
			// which may cause problems with the render
			m_textNode->setFont(m_font->getOsgFont());
			m_textNode->setText(m_text);
			m_textNode->setCharacterSize(m_characterSize);
			m_textNode->setMaximumWidth(m_optionalWidth.hasValue() ? m_optionalWidth.getValue() : 0.0);
			m_needUpdate = false;
		}
	}
	m_transform->setAttitude(osg::Quat(0.0, 0.0, 0.0, 1.0));
}

bool OsgTextRepresentation::doInitialize()
{
	bool result = true;

	// if the material was preassigned, don't create a default one
	if (getMaterial() == nullptr)
	{
		result = false;
		std::shared_ptr<OsgMaterial> material;
		material = buildMaterial("Shaders/unlit_texture.vert", "Shaders/unlit_text.frag");
		if (material != nullptr)
		{
			m_textNode->getOrCreateStateSet()->addUniform(new osg::Uniform("texture", 0));
			setMaterial(material);
			result = true;
		}
	}

	return result;

}

void OsgTextRepresentation::loadFont(std::string fileName)
{
	auto newFont = std::make_shared<OsgFont>();
	newFont->load(fileName);
	setFont(newFont);
}

void OsgTextRepresentation::setFont(std::shared_ptr<SurgSim::Framework::Asset> font)
{
	SURGSIM_ASSERT(! isInitialized()) << "Can't set font after text has been initialized.";
	SURGSIM_ASSERT(font != nullptr) << "Can't use nullptr font.";

	auto osgFont = std::dynamic_pointer_cast<OsgFont>(font);

	SURGSIM_ASSERT(osgFont != nullptr)
			<< "Font has to be OsgFont, instead it was " << font->getClassName();
	{
		boost::mutex::scoped_lock lock(m_parameterMutex);
		m_font = osgFont;
		m_needUpdate = true;
	}
}

std::shared_ptr<Font> OsgTextRepresentation::getFont() const
{
	return m_font;
}

void OsgTextRepresentation::setOptionalMaximumWidth(SurgSim::DataStructures::OptionalValue<double> maximum)
{
	boost::mutex::scoped_lock lock(m_parameterMutex);
	m_optionalWidth = maximum;
	m_needUpdate = true;
}

SurgSim::DataStructures::OptionalValue<double> OsgTextRepresentation::getOptionalMaximumWidth()
{
	return m_optionalWidth;
}

void OsgTextRepresentation::setMaximumWidth(double width)
{
	boost::mutex::scoped_lock lock(m_parameterMutex);
	if (width > 0.0)
	{
		m_optionalWidth.setValue(width);
	}
	else
	{
		m_optionalWidth.invalidate();
	}
	m_needUpdate = true;
}

double OsgTextRepresentation::getMaximumWidth()
{
	return m_optionalWidth.hasValue() ? m_optionalWidth.getValue() : 0.0;
}

void OsgTextRepresentation::setFontSize(double size)
{
	boost::mutex::scoped_lock lock(m_parameterMutex);
	m_characterSize = size;
	m_needUpdate = true;
}

double OsgTextRepresentation::getFontSize() const
{
	return m_characterSize;
}

void OsgTextRepresentation::setColor(SurgSim::Math::Vector4d color)
{
	m_textNode->setColor(toOsg(color));
}

SurgSim::Math::Vector4d  OsgTextRepresentation::getColor() const
{
	SurgSim::Math::Vector4d result = fromOsg(m_textNode->getColor()).cast<double>();
	return result;
}

}; // Graphics
}; // SurgSim
