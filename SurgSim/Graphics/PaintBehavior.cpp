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

#include <cmath>

#include "SurgSim/DataStructures/Image.h"
#include "SurgSim/DataStructures/ImageMap.h"
#include "SurgSim/Graphics/PaintBehavior.h"
#include "SurgSim/Math/Scalar.h"

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
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PaintBehavior, std::shared_ptr<Framework::Component>, Representation,
									  getRepresentation, setRepresentation);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PaintBehavior, Math::Vector4d, Color, getColor, setColor);
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
	boost::unique_lock<boost::mutex> lock(m_mutex);
	m_coordinates = coordinates;
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

	buildBrush(getRadius());

	return true;
}

void PaintBehavior::update(double dt)
{
	std::vector<Math::Vector2d> uvCoordinates;

	{
		boost::unique_lock<boost::mutex> lock(m_mutex);
		auto mesh = m_representation->getMesh();
		for (auto coordinate : m_coordinates)
		{
			if (coordinate.index < mesh->getNumVertices())
			{
				auto vertex1 = mesh->getVertex(mesh->getTriangle(coordinate.index).verticesId[0]);
				auto vertex2 = mesh->getVertex(mesh->getTriangle(coordinate.index).verticesId[1]);
				auto vertex3 = mesh->getVertex(mesh->getTriangle(coordinate.index).verticesId[2]);
				Math::Vector2d uv = vertex1.data.texture.getValue() * coordinate.coordinate[0] +
									vertex2.data.texture.getValue() * coordinate.coordinate[1] +
									vertex3.data.texture.getValue() * coordinate.coordinate[2];
				uvCoordinates.push_back(uv);
			}
		}
	}

	for (auto& uv : uvCoordinates)
	{
		Math::Vector2d xy = toPixel(uv);

		paint(xy);

		m_texture->getOsgTexture2d()->dirtyTextureObject();
	}
}

void PaintBehavior::setRadius(double radius)
{
		m_radius = radius;
}

double PaintBehavior::getRadius() const
{
	return m_radius;
}

void PaintBehavior::buildBrush(double radius)
{
	int brushWidth = static_cast<int>(ceil(2 * radius * m_width));
	int brushHeight = static_cast<int>(ceil(2 * radius * m_height));

	int centerX = brushWidth / 2;
	int centerY = brushHeight / 2;

	m_brushOffsetX = -1 * centerX;
	m_brushOffsetY = -1 * centerY;

	m_brush.resize(brushWidth, brushHeight);

	for (int i = 0; i < m_brush.cols(); i++)
	{
		for (int j = 0; j < m_brush.rows(); j++)
		{
			double mag = ((i - centerX) * (i - centerX)) + ((j - centerY) * (j - centerY));
			if (sqrt(mag) < m_brush.cols() / 2.0)
			{
				m_brush(i, j) = 1.0;
			}
			else
			{
				m_brush(i, j) = 0.0;
			}
		}
	}
}

void PaintBehavior::buildAntiAliasedBrush(double radius)
{
	static const double sqrt2Half = 0.7071067811865475;
	double dist;
	int brushWidth = static_cast<int>(ceil(2 * radius * m_width));
	int brushHeight = static_cast<int>(ceil(2 * radius * m_height));

	int centerX = brushWidth / 2;
	int centerY = brushHeight / 2;

	m_brushOffsetX = -1 * centerX;
	m_brushOffsetY = -1 * centerY;

	m_brush.resize(brushWidth, brushHeight);

	for (int i = 0; i < m_brush.cols(); i++)
	{
		for (int j = 0; j < m_brush.rows(); j++)
		{
			double mag = (i - centerX) * (i - centerX) + (j - centerY) * (j - centerY);
			dist = (m_brush.cols() / 2) - sqrt(mag);
			dist /= m_brush.cols() / 2;
			if (dist > sqrt2Half)
			{
				m_brush(i, j) = 1.0;
			}
			else if (dist < 0.0)
			{
				m_brush(i, j) = 0.0;
			}
			else
			{
				m_brush(i, j) = sqrt(2) * dist;
			}
		}
	}
}

Math::Vector2d PaintBehavior::toPixel(const Math::Vector2d& uv)
{
	double s = uv[0];
	double t = uv[1];

	s = Math::clamp(s, 0.0, 1.0, 0.0);
	t = Math::clamp(t, 0.0, 1.0, 0.0);

	Math::Vector2d xy;
	xy << floor(s * m_width + 0.5), floor(t * m_height + 0.5);

	return xy;
}

void PaintBehavior::paint(const Math::Vector2d& coordinates)
{
	size_t numChannels = 4;

	DataStructures::ImageMap<unsigned char> image(m_width,
												  m_height,
												  numChannels,
												  m_texture->getOsgTexture2d()->getImage()->data());

	size_t topLeftX = static_cast<size_t>(std::max(0.0, coordinates[0] + m_brushOffsetX));
	size_t topLeftY = static_cast<size_t>(std::max(0.0, coordinates[1] + m_brushOffsetY));
	size_t bottomRightX = std::min(m_width - topLeftX, static_cast<size_t>(m_brush.cols()));
	size_t bottomRightY = std::min(m_height - topLeftY, static_cast<size_t>(m_brush.rows()));
	for (size_t channel = 0; channel < numChannels; channel++)
	{
		auto imageBlock = image.getChannel(channel).block(topLeftX, topLeftY, bottomRightX, bottomRightY);
		auto brushBlock = m_brush.topLeftCorner(bottomRightX, bottomRightY);
		imageBlock = (brushBlock.array() > 0).select((brushBlock * m_color(channel)*255).template cast<unsigned char>(),
						imageBlock);
	}
}

} // Graphics
} // SurgSim