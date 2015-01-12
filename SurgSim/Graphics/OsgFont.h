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

#ifndef SURGSIM_GRAPHICS_OSGFONT_H
#define SURGSIM_GRAPHICS_OSGFONT_H

#include "SurgSim/Graphics/Font.h"

#include <osg/ref_ptr>

namespace osgText
{
class Font;
}

namespace SurgSim
{
namespace Graphics
{

/// Osg specialization of the Font class, supports osgText::Font
class OsgFont : public SurgSim::Graphics::Font
{
public:
	/// Constructor
	OsgFont();

	/// Destructor
	virtual ~OsgFont();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgFont);

	/// \return the appropriate osgText::Font reference
	osg::ref_ptr<osgText::Font> getOsgFont() const;


protected:

	bool doLoad(const std::string& filePath) override;

private:

	/// Osg pointer to the font structure
	osg::ref_ptr<osgText::Font> m_font;

};

}
}

#endif
