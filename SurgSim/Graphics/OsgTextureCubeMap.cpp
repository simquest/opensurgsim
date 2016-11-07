// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Graphics/OsgTextureCubeMap.h"

#include <osgDB/ReadFile>

using SurgSim::Graphics::OsgTextureCubeMap;


OsgTextureCubeMap::OsgTextureCubeMap() : OsgTexture(new osg::TextureCubeMap())
{

}

void OsgTextureCubeMap::setSize(int width, int height)
{
	getOsgTextureCubeMap()->setTextureSize(width, height);
}

void OsgTextureCubeMap::getSize(int* width, int* height) const
{
	*width = getOsgTextureCubeMap()->getTextureWidth();
	if (*width == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*width = getOsgTexture()->getImage(0)->s();
	}
	*height = getOsgTextureCubeMap()->getTextureHeight();
	if (*height == 0 && getOsgTexture()->getNumImages() > 0)
	{
		*height = getOsgTexture()->getImage(0)->t();
	}
}

bool OsgTextureCubeMap::loadImage(const std::string& filePath)
{
	osg::ref_ptr<osg::Image> imageNode = osgDB::readImageFile(filePath);
	if (! imageNode.valid())
	{
		return false;
	}

	// Split up image to the six sides of the cube
	int width = imageNode->s() / 3;
	int height = imageNode->t() / 4;

	osg::ref_ptr<osg::Image> negativeZ = copyImageBlock(*imageNode, width, 0, width, height);
	negativeZ->flipHorizontal();

	osg::ref_ptr<osg::Image> negativeY = copyImageBlock(*imageNode, width, height, width, height);
	negativeY->flipVertical();

	osg::ref_ptr<osg::Image> negativeX = copyImageBlock(*imageNode, 0, height * 2, width, height);
	negativeX->flipVertical();

	osg::ref_ptr<osg::Image> positiveZ = copyImageBlock(*imageNode, width, height * 2, width, height);
	positiveZ->flipVertical();

	osg::ref_ptr<osg::Image> positiveX = copyImageBlock(*imageNode, width * 2, height * 2, width, height);
	positiveX->flipVertical();

	osg::ref_ptr<osg::Image> positiveY = copyImageBlock(*imageNode, width, height * 3, width, height);
	positiveY->flipVertical();

	getOsgTexture()->setImage(osg::TextureCubeMap::POSITIVE_X, positiveX);
	getOsgTexture()->setImage(osg::TextureCubeMap::NEGATIVE_X, negativeX);
	getOsgTexture()->setImage(osg::TextureCubeMap::POSITIVE_Y, positiveY);
	getOsgTexture()->setImage(osg::TextureCubeMap::NEGATIVE_Y, negativeY);
	getOsgTexture()->setImage(osg::TextureCubeMap::POSITIVE_Z, positiveZ);
	getOsgTexture()->setImage(osg::TextureCubeMap::NEGATIVE_Z, negativeZ);

	return true;
}

bool OsgTextureCubeMap::loadImageFaces(const std::string& negativeX, const std::string& positiveX,
									   const std::string& negativeY, const std::string& positiveY,
									   const std::string& negativeZ, const std::string& positiveZ)
{
	osg::ref_ptr<osg::Image> negativeXImage = osgDB::readImageFile(negativeX);
	osg::ref_ptr<osg::Image> positiveXImage = osgDB::readImageFile(positiveX);
	osg::ref_ptr<osg::Image> negativeYImage = osgDB::readImageFile(negativeY);
	osg::ref_ptr<osg::Image> positiveYImage = osgDB::readImageFile(positiveY);
	osg::ref_ptr<osg::Image> negativeZImage = osgDB::readImageFile(negativeZ);
	osg::ref_ptr<osg::Image> positiveZImage = osgDB::readImageFile(positiveZ);

	if (negativeXImage.valid() && positiveXImage.valid() && negativeYImage.valid() && positiveYImage.valid() &&
		negativeZImage.valid() && positiveZImage.valid())
	{
		getOsgTexture()->setImage(osg::TextureCubeMap::NEGATIVE_X, negativeXImage);
		getOsgTexture()->setImage(osg::TextureCubeMap::POSITIVE_X, positiveXImage);
		getOsgTexture()->setImage(osg::TextureCubeMap::NEGATIVE_Y, negativeYImage);
		getOsgTexture()->setImage(osg::TextureCubeMap::POSITIVE_Y, positiveYImage);
		getOsgTexture()->setImage(osg::TextureCubeMap::NEGATIVE_Z, negativeZImage);
		getOsgTexture()->setImage(osg::TextureCubeMap::POSITIVE_Z, positiveZImage);
		return true;
	}
	else
	{
		return false;
	}
}

osg::ref_ptr<osg::Image> OsgTextureCubeMap::copyImageBlock(
	const osg::Image& source,
	size_t startColumn, size_t startRow,
	size_t width, size_t height)
{
	size_t pixelSize = source.getPixelSizeInBits() / 8;

	unsigned char* buffer = new unsigned char[(width * height) * pixelSize];

	size_t index = 0;

	for (size_t row = startRow; row < startRow + height; ++row)
	{
		for (size_t column = startColumn; column < startColumn + width; ++column)
		{
			const unsigned char* pixel = source.data(column, row, 0);

			for (size_t p = 0; p < pixelSize; ++p)
			{
				buffer[index] = pixel[p];
				index++;
			}
		}
	}

	osg::ref_ptr<osg::Image> subImage = new osg::Image();
	subImage->setImage(width, height, 1, source.getInternalTextureFormat(), source.getPixelFormat(),
					   GL_UNSIGNED_BYTE, buffer, osg::Image::USE_NEW_DELETE, 1);

	return subImage;
}

osg::ref_ptr<osg::TextureCubeMap> SurgSim::Graphics::OsgTextureCubeMap::getOsgTextureCubeMap() const
{
	return static_cast<osg::TextureCubeMap*>(getOsgTexture().get());
}
