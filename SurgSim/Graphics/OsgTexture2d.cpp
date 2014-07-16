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

#include "SurgSim/Graphics/OsgTexture2d.h"

using SurgSim::Graphics::OsgTexture2d;

OsgTexture2d::OsgTexture2d() : OsgTexture(new osg::Texture2D())
{
}

void OsgTexture2d::setSize(int width, int height)
{
	getOsgTexture2d()->setTextureSize(width, height);
}

void OsgTexture2d::getSize(int* width, int* height) const
{
	*width = getOsgTexture2d()->getTextureWidth();
	if (*width == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*width = getOsgTexture()->getImage(0)->s();
	}
	*height = getOsgTexture2d()->getTextureHeight();
	if (*height == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*height = getOsgTexture()->getImage(0)->t();
	}
}

template <>
std::shared_ptr<SurgSim::Graphics::OsgTexture2d>
SurgSim::Framework::convert(boost::any val)
{
	std::shared_ptr<SurgSim::Graphics::OsgTexture2d> result;
	try
	{
		std::shared_ptr<SurgSim::Graphics::Texture> tex =
			boost::any_cast<std::shared_ptr<SurgSim::Graphics::Texture>>(val);
		result = std::dynamic_pointer_cast<SurgSim::Graphics::OsgTexture2d>(tex);
	}
	catch (boost::bad_any_cast&)
	{
		result = boost::any_cast<std::shared_ptr<SurgSim::Graphics::OsgTexture2d>>(val);
	}

	SURGSIM_ASSERT(result != nullptr)
			<< "Conversion from the incoming type to OsgTexture2d failed, this probably means that the input value "
			<< "was a texture but not an OsgTexture2d.";

	return result;
}
