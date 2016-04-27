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

#include <math.h>

#include "SurgSim/DataStructures/Image.h"
#include "SurgSim/Graphics/PaintBehavior.h"


namespace SurgSim
{
namespace Graphics
{
PaintBehavior::PaintBehavior(const std::string& name) :
	Framework::Behavior(name),
	m_width(0),
	m_height(0),
	m_radius(0)
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

void PaintBehavior::setColor(const Math::Vector4d& color)
{
	m_color = color;
}

Math::Vector4d PaintBehavior::getColor() const
{
	return m_color;
}

void PaintBehavior::setCoordinates(const std::vector<DataStructures::IndexedLocalCoordinate>& coordinates)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	auto mesh = m_representation->getMesh();
	for (auto coordinate : coordinates)
	{
		auto vertex1 = mesh->getVertex(mesh->getTriangle(coordinate.index).verticesId[0]);
		auto vertex2 = mesh->getVertex(mesh->getTriangle(coordinate.index).verticesId[1]);
		auto vertex3 = mesh->getVertex(mesh->getTriangle(coordinate.index).verticesId[2]);
		Math::Vector2d uv = vertex1.data.texture.getValue() * coordinate.coordinate[0] +
							vertex2.data.texture.getValue() * coordinate.coordinate[1] +
							vertex3.data.texture.getValue() * coordinate.coordinate[2];
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

	if (m_radius <= 0.0 || m_radius > 1.0)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger()) << getClassName() << " named " << getName() <<
									" does not have a radius set";
		return false;
	}

	m_texture->getOsgTexture2d()->setSourceFormat(GL_RGBA);
	m_texture->getOsgTexture2d()->setSourceType(GL_BYTE);

	m_texture->getOsgTexture2d()->dirtyTextureObject();

	m_texture->getSize(&m_width, &m_height);

	for (size_t i = 0; i < m_width; i++)
	{
		for (size_t j = 0; j < m_height; j++)
		{
			auto data = m_texture->getOsgTexture2d()->getImage()->data();
			auto ptr = data + (j * m_width + i) * 4;
			*(ptr++) = 0;
			*(ptr++) = 0;
			*(ptr++) = 0;
			*(ptr++) = 0;
		}
	}
	m_texture->getOsgTexture2d()->dirtyTextureObject();

	buildBrush(getRadius());

	return true;
}

void PaintBehavior::update(double dt)
{
	std::vector<Math::Vector2d> newCoordinates;
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		std::swap(newCoordinates, m_paintCoordinates);
	}

	for (Math::Vector2d uv : newCoordinates)
	{
		Math::Vector2d xy = toPixel(uv);

		paint(xy);

		m_texture->getOsgTexture2d()->dirtyTextureObject();
	}
}

void PaintBehavior::setRadius(double radius)
{
	if (radius > 0.0 && radius < 1.0)
	{
		m_radius = radius;
	}
}

double PaintBehavior::getRadius() const {
	return m_radius;
}

void PaintBehavior::buildBrush(double radius) {
	m_brushWidth = static_cast<int>(ceil(2 * radius * m_width));
	m_brushHeight = static_cast<int>(ceil(2 * radius * m_height));

	int centerX = m_brushWidth / 2;
	int centerY = m_brushHeight / 2;

	m_brushOffsetX = -1 * centerX;
	m_brushOffsetY = -1 * centerY;

	m_brush.resize(m_brushWidth, m_brushHeight);

	for (int i = 0; i < m_brushWidth; i++)
	{
		for (int j = 0; j < m_brushHeight; j++)
		{
			double mag = ((i - centerX) * (i - centerX)) + ((j - centerY) * (j - centerY));
			if (sqrt(mag) < m_brushWidth / 2.0)
			{
				m_brush(i * m_brushHeight + j) = 1.0;
			}
			else
			{
				m_brush(i * m_brushHeight + j) = 0.0;
			}
		}
	}
}

void PaintBehavior::buildAntiAliasedBrush(double radius) {
	static const double sqrt2Half = 0.7071067811865475;
	double dist;
	m_brushWidth = static_cast<int>(ceil(2 * radius * m_width));
	m_brushHeight = static_cast<int>(ceil(2 * radius * m_height));

	int centerX = m_brushWidth / 2;
	int centerY = m_brushHeight / 2;

	m_brushOffsetX = -1 * centerX;
	m_brushOffsetY = -1 * centerY;

	m_brush.resize(m_brushWidth, m_brushHeight);

	for (int i = 0; i < m_brushWidth; i++)
	{
		for (int j = 0; j < m_brushHeight; j++)
		{
			double mag = (i - centerX) * (i - centerX) + (j - centerY) * (j - centerY);
			dist = (m_brushWidth / 2) - sqrt(mag);
			dist /= m_brushWidth / 2;
			if (dist > sqrt2Half)
			{
				m_brush(i * m_brushHeight + j) = 1.0;
			}
			else if (dist < 0.0)
			{
				m_brush(i * m_brushHeight + j) = 0.0;
			}
			else
			{
				m_brush(i * m_brushHeight + j) = sqrt(2) * dist;
			}
		}
	}
}

Math::Vector2d PaintBehavior::toPixel(Math::Vector2d uv) {
	double s = uv[0];
	double t = uv[1];

	if (s > 1.0f)
	{
		s = 1.0f;
	}
	else if (s < 0.0f)
	{
		s = 0.0f;
	}

	if (t > 1.0f)
	{
		t = 1.0f;
	}
	else if (t < 0.0f)
	{
		t = 0.0f;
	}

	Math::Vector2d xy;
	xy << round(s * m_width), round(t * m_height);

	return xy;
}

void PaintBehavior::paint(Math::Vector2d coordinates) {
	int numChannels = 4;

	auto data = m_texture->getOsgTexture2d()->getImage()->data();

	for (size_t x = 0; x < m_brushWidth; x++)
	{
		for (size_t y = 0; y < m_brushHeight; y++)
		{
			if (m_brush(x * m_brushHeight + y) > 0.0f)
			{
				size_t i = static_cast<size_t>(coordinates[0] + m_brushOffsetX + x);
				size_t j = static_cast<size_t>(coordinates[1] + m_brushOffsetY + y);
				if (i >= 0 && i < m_width && j >= 0 && j < m_height)
				{
					Eigen::Map<Eigen::Matrix<unsigned char, 4, 1>> pixel(data + (j * m_width + i) * numChannels);
					pixel = (m_brush(x * m_brushHeight + y) * m_color * 255).template cast<unsigned char>();
				}
			}
		}
	}
}

} // Graphics
} // SurgSim