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

#ifndef SURGSIM_GRAPHICS_OSGAXESREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGAXESREPRESENTATION_H

#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/AxesRepresentation.h"
#include "SurgSim/Graphics/OsgUnitAxes.h"


#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{
namespace Graphics
{

SURGSIM_STATIC_REGISTRATION(OsgAxesRepresentation);

/// Osg axes representation implementation for the AxesRepresentation interface in graphics.
class OsgAxesRepresentation : public OsgRepresentation, public AxesRepresentation
{
public:

	/// Constructor
	explicit OsgAxesRepresentation(const std::string& name);
	~OsgAxesRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgAxesRepresentation);

	/// Sets the size of the shown axes.
	/// \param	val	The value.
	void setSize(double val) override;

	/// Gets the current size.
	/// \return	The size.
	double getSize() override;

private:

	/// Shared unit axes, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitAxes> m_sharedUnitAxes;

	/// Returns the shared unit axes
	static std::shared_ptr<OsgUnitAxes> getShareUnitAxes();

	/// Hold the size
	double m_size;
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#endif