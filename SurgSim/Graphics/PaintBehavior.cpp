// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/Image.h"
#include "SurgSim/Graphics/PaintBehavior.h"


namespace SurgSim
{
namespace Graphics
{
PaintBehavior::PaintBehavior(const std::string& name) :
	Framework::Behavior(name), m_s(0.0), m_t(0.0), m_width(0), m_height(0)
{
}

void PaintBehavior::setPainter(std::shared_ptr<Collision::Representation> painter)
{
	m_painter = painter;
}

std::shared_ptr<Collision::Representation> PaintBehavior::getPainter() const
{
	return m_painter;
}

void PaintBehavior::setRepresentation(std::shared_ptr<Graphics::OsgMeshRepresentation> representation) {
	m_representation = representation;
}

std::shared_ptr<Graphics::OsgMeshRepresentation> PaintBehavior::getRepresentation() const {
	return m_representation;
}

void PaintBehavior::setTexture(std::shared_ptr<Graphics::OsgTexture2d> texture)
{
	m_texture = texture;
}

std::shared_ptr<Graphics::OsgTexture2d> PaintBehavior::getTexture() const
{
	return m_texture;
}

void PaintBehavior::setPaintColor(Math::Vector4d color)
{
	m_color = color;
}

Math::Vector4d PaintBehavior::getPaintColor() const
{
	return m_color;
}

void PaintBehavior::setPaintCoordinate(std::vector<DataStructures::IndexedLocalCoordinate> coordinates) {
	auto vertex1 = m_representation->getMesh()->getVertex(coordinates[0].index);
	auto vertex2 = m_representation->getMesh()->getVertex(coordinates[1].index);
	auto vertex3 = m_representation->getMesh()->getVertex(coordinates[2].index);
	Math::Vector2d uv = vertex1.data.texture.getValue() * coordinates[0].coordinate[0] +
						   vertex2.data.texture.getValue() * coordinates[1].coordinate[0] +
						   vertex3.data.texture.getValue() * coordinates[2].coordinate[0];
	m_s = uv[0];
	m_t = uv[1];
}

bool PaintBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_representation != nullptr) << "You must provide a graphics representation to pain into.";
	SURGSIM_ASSERT(m_painter != nullptr) << "You must provide a collision representation that will paint the decals.";
	SURGSIM_ASSERT(m_texture != nullptr) << "You must provide a texture to paint decals onto.";

	m_texture->getOsgTexture2d()->setSourceFormat(GL_RGBA);
	m_texture->getOsgTexture2d()->setSourceType(GL_BYTE);

	m_texture->getSize(&m_width, &m_height);

	return true;
}

bool PaintBehavior::doWakeUp()
{
	return true;
}

void PaintBehavior::update(double dt)
{
	m_s += 0.005f;
	m_t += 0.003f;

	if (m_s > 1.0f)
	{
		m_s -= 1.0f;
	}

	if (m_t > 1.0f)
	{
		m_t -= 1.0f;
	}

	unsigned coordinateX = (unsigned)(m_s * m_width);
	unsigned coordinateY = (unsigned)(m_t * m_height);

	int numChannels = m_texture->getOsgTexture2d()->getImage()->getPixelSizeInBits() / 8;

	auto data = m_texture->getOsgTexture2d()->getImage()->data();
	unsigned char* ptr = &data[(coordinateY * m_width * numChannels) + (coordinateX * numChannels)];
	*(ptr++) = (unsigned char)(m_color[0] * 255);
	*(ptr++) = (unsigned char)(m_color[1] * 255);
	*(ptr++) = (unsigned char)(m_color[2] * 255);
	if (numChannels == 4)
	{
		*(ptr++) = (unsigned char)(m_color[3] * 255);
	}

	m_texture->getOsgTexture2d()->getImage()->dirty();
}


} // Graphics
} // SurgSim