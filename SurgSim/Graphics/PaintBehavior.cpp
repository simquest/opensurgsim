// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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
	Framework::Behavior(name),
	m_width(0),
	m_height(0)
{
}

void PaintBehavior::setRepresentation(std::shared_ptr<Framework::Component> representation)
{
	m_representation = Framework::checkAndConvert<Graphics::OsgMeshRepresentation>(representation,
																				   "SurgSim::Graphics::Representation");
}

std::shared_ptr<Graphics::OsgMeshRepresentation> PaintBehavior::getRepresentation() const
{
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
	auto mesh = m_representation->getMesh();
	for (size_t i = 0; i < coordinates.size(); i++)
	{
		auto vertex1 = mesh->getVertex(mesh->getTriangle(coordinates[i].index).verticesId[0]);
		auto vertex2 = mesh->getVertex(mesh->getTriangle(coordinates[i].index).verticesId[1]);
		auto vertex3 = mesh->getVertex(mesh->getTriangle(coordinates[i].index).verticesId[2]);
		Math::Vector2d uv = vertex1.data.texture.getValue() * coordinates[i].coordinate[0] +
							vertex2.data.texture.getValue() * coordinates[i].coordinate[1] +
							vertex3.data.texture.getValue() * coordinates[i].coordinate[2];
		m_paintCoordinates.push_back(uv);
	}
}

bool PaintBehavior::doInitialize()
{
	return true;
}

bool PaintBehavior::doWakeUp()
{
	if (m_representation == nullptr)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger()) << getClassName() << " named " << getName() <<
									 " does not have a graphics representation to paint into";
		return false;
	}

	if (m_texture == nullptr)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger()) << getClassName() << " named " << getName() <<
									 " does not have a texture to paint onto";
		return false;
	}

	m_texture->getOsgTexture2d()->setSourceFormat(GL_RGBA);
	m_texture->getOsgTexture2d()->setSourceType(GL_BYTE);
	m_texture->getOsgTexture2d()->dirtyTextureObject();

	m_texture->getSize(&m_width, &m_height);

	return true;
}

void PaintBehavior::update(double dt)
{
	for (Math::Vector2d uv : m_paintCoordinates) {

		double s = uv[0];
		double t = uv[1];


		if (s > 1.0f) {
			s = 1.0f;
		}

		if (t > 1.0f) {
			t = 1.0f;
		}

		size_t coordinateX = static_cast<size_t>(s * m_width);
		size_t coordinateY = static_cast<size_t>(t * m_height);

		int numChannels = 4;

		auto data = m_texture->getOsgTexture2d()->getImage()->data();
		Eigen::Map<Eigen::Matrix<unsigned char, 4, 1>> pixel(data + (coordinateY * m_width * numChannels) +
																	 (coordinateX * numChannels));
		pixel = (m_color * 255).template cast<unsigned char>();

		m_texture->getOsgTexture2d()->dirtyTextureObject();

		m_paintCoordinates.clear();
	}
}


} // Graphics
} // SurgSim