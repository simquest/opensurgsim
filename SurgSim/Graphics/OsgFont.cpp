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

#include "SurgSim/Graphics/OsgFont.h"

#include <osgText/Font>

#include "SurgSim/Framework/Log.h"

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Graphics::OsgFont, OsgFont);

namespace SurgSim
{
namespace Graphics
{


OsgFont::OsgFont()
{
	m_font = osgText::Font::getDefaultFont();
}

OsgFont::~OsgFont()
{

}

bool OsgFont::doLoad(const std::string& filePath)
{
	std::string osgFilePath = osgText::findFontFile(filePath);

	m_font = osgText::readFontFile(osgFilePath);
	bool result = m_font.valid();
	if (!result)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Graphics"))
				<< "The font could not be loaded from " << osgFilePath << ".";
		m_font = osgText::Font::getDefaultFont();
	}

	return result;
}

osg::ref_ptr<osgText::Font> OsgFont::getOsgFont() const
{
	return m_font;
}

}
}

