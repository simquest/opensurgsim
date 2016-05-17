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

#ifndef SURGSIM_GRAPHICS_PAINTBEHAVIOR_H
#define SURGSIM_GRAPHICS_PAINTBEHAVIOR_H

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

/// Behavior class to allow a specified scene element to receive painting effects
class PaintBehavior : public Framework::Behavior
{
public:
	explicit PaintBehavior(const std::string& name);

	/// Sets graphics representation being painted on
	/// \param representation Graphics representation pointer
	void setRepresentation(std::shared_ptr<Framework::Component> representation);

	/// Gets graphics representation being painted on
	/// \return Shared pointer to a graphics representation
	std::shared_ptr<Graphics::OsgMeshRepresentation> getRepresentation() const;

	/// Sets the texture layer to paint onto
	/// \param texture 2d texture from graphics representation
	void setTexture(std::shared_ptr<Graphics::OsgTexture2d> texture);

	/// Gets 2d texture that is being painted on
	/// \return Shared pointer to 2d texture
	std::shared_ptr<Graphics::OsgTexture2d> getTexture() const;

	/// Sets color of the paint
	/// \param color RGBA color in [0-1] range
	void setColor(const Math::Vector4d& color);

	/// Gets color of the paint
	/// \return Vector4d representation of RGBA color in [0-1] range
	Math::Vector4d getColor() const;

	/// Sets radius of paint splat
	/// \param  radius Radius in texture coordinate range [0-1]
	void setRadius(double radius);

	/// Gets radius of paint splat
	/// \return Radius in texture coordinate range [0-1]
	double getRadius() const;

	/// Sets collection of local triangle coordinates to paint on during next update
	/// \param coordinate Standard vector of IndexedLocalCoordinates
	void setCoordinates(const std::vector<DataStructures::IndexedLocalCoordinate>& coordinate);

	bool doInitialize() override;
	bool doWakeUp() override;

	void update(double dt) override;

private:

	/// Builds paint brush at the set radius size
	void buildBrush(double radius);

	/// Builds an antialiased brush at the set radius size
	void buildAntiAliasedBrush(double radius);

	/// Convert texture uv coordinates to pixel coordinates
	Math::Vector2d toPixel(const Math::Vector2d& uv);

	/// Apply paint brush to texture at specified texture coordinates
	void paint(const Math::Vector2d& coordinates);

	/// Graphics representation of the mesh to apply behavior to
	std::shared_ptr<Graphics::OsgMeshRepresentation> m_representation;

	/// Image data of the texture to be used as the decal layer
	std::shared_ptr<Graphics::OsgTexture2d> m_texture;

	/// Color to use for decal painting
	Math::Vector4d m_color;

	/// Collection of UV texture coordinates to paint to on next update
	std::vector<DataStructures::IndexedLocalCoordinate> m_coordinates;

	/// Width of assigned texture
	int m_width;

	/// Height of assigned texture
	int m_height;

	int m_brushOffsetX;
	int m_brushOffsetY;

	/// Radius of brush
	double m_radius;

	Math::Matrix m_brush;

	boost::mutex m_mutex;
};

} // Graphics
} // SurgSim

#endif // SURGSIM_GRAPHICS_PAINTBEHAVIOR_H