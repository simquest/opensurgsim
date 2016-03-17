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

#ifndef SURGSIM_GRAPHICS_STAIN_COMPONENT_H
#define SURGSIM_GRAPHICS_STAIN_COMPONENT_H

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

/// Component class to add to an element to allow it to receive staining
class DecalBehavior : public Framework::Behavior
{
public:
	explicit DecalBehavior(const std::string& name);

	void setPainter(std::shared_ptr<Collision::Representation> painter);
	std::shared_ptr<Collision::Representation> getPainter() const;

	void setTexture(std::shared_ptr<Graphics::OsgTexture2d> texture);
	std::shared_ptr<Graphics::OsgTexture2d> getTexture() const;

	void setDecalColor(Math::Vector4f color);
	Math::Vector4f getDecalColor() const;

	bool doInitialize() override;
	bool doWakeUp() override;

	void update(double dt);

private:

	/// Representation to filter for to trigger staining
	std::shared_ptr<Collision::Representation> m_painter;

	/// Image data of the texture to be used as the decal layer
	std::shared_ptr<Graphics::OsgTexture2d> m_texture;

	/// Color to use for decal painting
	Math::Vector4f m_color;

	float s;
	float t;

};

} // Graphics
} // SurgSim

#endif // SURGSIM_GRAPHICS_STAIN_COMPONENT_H