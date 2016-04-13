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

#ifndef SURGSIM_GRAPHICS_PAINT_BEHAVIOR_H
#define SURGSIM_GRAPHICS_PAINT_BEHAVIOR_H

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

/// Component class to add to an element to allow it to receive staining
class PaintBehavior : public Framework::Behavior
{
public:
	explicit PaintBehavior(const std::string& name);

	void setPainter(std::shared_ptr<Collision::Representation> painter);
	std::shared_ptr<Collision::Representation> getPainter() const;

	void setRepresentation(std::shared_ptr<Graphics::OsgMeshRepresentation> representation);
	std::shared_ptr<Graphics::OsgMeshRepresentation> getRepresentation() const;

	void setTexture(std::shared_ptr<Graphics::OsgTexture2d> texture);
	std::shared_ptr<Graphics::OsgTexture2d> getTexture() const;

	void setPaintColor(Math::Vector4d color);
	Math::Vector4d getPaintColor() const;

	void setPaintCoordinate(std::vector<DataStructures::IndexedLocalCoordinate> coordinate);

	bool doInitialize() override;
	bool doWakeUp() override;

	void update(double dt);

private:

	/// Representation to filter for to trigger staining
	std::shared_ptr<Collision::Representation> m_painter;

	std::shared_ptr<Graphics::OsgMeshRepresentation> m_representation;

	/// Image data of the texture to be used as the decal layer
	std::shared_ptr<Graphics::OsgTexture2d> m_texture;

	/// Color to use for decal painting
	Math::Vector4d m_color;

	double m_s;
	double m_t;
	int m_width;
	int m_height;

};

} // Graphics
} // SurgSim

#endif // SURGSIM_GRAPHICS_PAINT_BEHAVIOR_H