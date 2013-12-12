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

#include "SurgSim/Graphics/OsgTexture3d.h"

#include "SurgSim/Framework/Assert.h"

#include <osgDB/ReadFile>

using SurgSim::Graphics::OsgTexture3d;

OsgTexture3d::OsgTexture3d() : OsgTexture(new osg::Texture3D())
{
}

void OsgTexture3d::setSize(int width, int height, int depth)
{
	getOsgTexture3d()->setTextureSize(width, height, depth);
}

void OsgTexture3d::getSize(int* width, int* height, int* depth) const
{
	*width = getOsgTexture3d()->getTextureWidth();
	if (*width == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*width = getOsgTexture()->getImage(0)->s();
	}
	*height = getOsgTexture3d()->getTextureHeight();
	if (*height == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*height = getOsgTexture()->getImage(0)->t();
	}
	*depth = getOsgTexture3d()->getTextureDepth();
	if (*depth == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*depth = getOsgTexture()->getImage(0)->r();
	}
}

bool OsgTexture3d::loadImageSlices(const std::vector<std::string>& filePaths)
{
	SURGSIM_ASSERT(! filePaths.empty()) << "No image file paths to load!";

	std::vector<osg::ref_ptr<osg::Image>> slices;
	slices.reserve(filePaths.size());

	int width;
	int height;
	GLenum pixelFormat;
	GLenum dataType;

	/// Load first slice, all others should have same properties
	{
		osg::ref_ptr<osg::Image> firstSlice = osgDB::readImageFile(filePaths.front());
		if (! firstSlice.valid())
		{
			return false;
		}
		slices.push_back(firstSlice);
		width = firstSlice->s();
		height = firstSlice->t();
		pixelFormat = firstSlice->getPixelFormat();
		dataType = firstSlice->getDataType();
	}

	/// Load the remaining slices
	for (auto it = filePaths.begin() + 1; it != filePaths.end(); ++it)
	{
		osg::ref_ptr<osg::Image> slice = osgDB::readImageFile(*it);
		if (! slice.valid())
		{
			return false;
		}
		SURGSIM_ASSERT(slice->s() == width) << "Slice has different width! File: " << *it <<
			" Width: " << slice->s() << " Expected Width: " << width;
		SURGSIM_ASSERT(slice->t() == height) << "Slice has different height! File: " << *it <<
			" Height: " << slice->s() << " Expected Height: " << height;
		SURGSIM_ASSERT(slice->getPixelFormat() == pixelFormat) << "Slice has different pixel format! File: " <<
			*it << " Pixel Format: " << slice->getPixelFormat() << " Expected Pixel Format: " << pixelFormat;
		SURGSIM_ASSERT(slice->getDataType() == dataType) << "Slice has different data type! File: " <<
			*it << " Data type: " << slice->getDataType() << " Expected Data Type: " << dataType;
		slices.push_back(slice);
	}

	/// Copy the slices into the 3D image, starting at depth 0
	osg::ref_ptr<osg::Image> image = new osg::Image();
	image->allocateImage(width, height, slices.size(), pixelFormat, dataType);

	int depth = 0;
	for (auto it = slices.begin(); it != slices.end(); ++it)
	{
		image->copySubImage(0, 0, depth, it->get());
	}

	getOsgTexture()->setImage(0u, image);

	return true;
}