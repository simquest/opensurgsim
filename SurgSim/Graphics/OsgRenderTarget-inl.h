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

#ifndef SURGSIM_GRAPHICS_OSGRENDERTARGET_INL_H
#define SURGSIM_GRAPHICS_OSGRENDERTARGET_INL_H

// OSG supports 16 color textures as of now + depth and stencil
const int OsgSupportedTextureCount = 16 + 2;

template <class T>
OsgRenderTarget<T>::OsgRenderTarget() :
	m_width(0),
	m_height(0),
	m_scale(1.0),
	m_colorTargetCount(0),
	m_textures(OsgSupportedTextureCount)
{
}

template <class T>
OsgRenderTarget<T>::OsgRenderTarget(
		int width,
		int height,
		double scale,
		int colorCount,
		bool useDepth,
		bool useStencil) :
	m_scale(scale),
	m_width(width),
	m_height(height),
	m_textures(OsgSupportedTextureCount),
	m_colorTargetCount(0)
{
	setColorTargetCount(colorCount);
	useDepthTarget(useDepth);
	useStencilTarget(useStencil);
}


template <class T>
OsgRenderTarget<T>::~OsgRenderTarget()
{
}

template <class T>
void OsgRenderTarget<T>::setSize(int width, int height)
{
	m_width = width;
	m_height = height;
	rebuildTextures();
}

template <class T>
void OsgRenderTarget<T>::getSize(int* width, int* height) const
{
	*width = m_width;
	*height = m_height;
}

template <class T>
void OsgRenderTarget<T>::setScale(double scale)
{
	m_scale = scale;
	rebuildTextures();
}

template <class T>
double OsgRenderTarget<T>::getScale() const
{
	return m_scale;
}


template <class T>
int OsgRenderTarget<T>::setColorTargetCount(int count)
{
	int result = (count < 16) ? count : 16;

	// Keep the other texture allocated when the count goes down
	// Rendertargets are probably not going to change that much once set up
	// #memory
	for (int i = m_colorTargetCount; i<result; ++i)
	{
		setupTexture(TARGETTYPE_COLORBASE+i);
	}
	m_colorTargetCount = result;
	return result;
}

template <class T>
int OsgRenderTarget<T>::getColorTargetCount() const
{
	return m_colorTargetCount;
}

template <class T>
std::shared_ptr<Texture> OsgRenderTarget<T>::getColorTarget(int index) const
{
	std::shared_ptr<Texture> result;

	if (index < m_colorTargetCount)
	{
		result = m_textures[TARGETTYPE_COLORBASE + index];
	}

	return result;
}

template <class T>
std::shared_ptr<T> OsgRenderTarget<T>::getColorTargetOsg(int index) const
{
	std::shared_ptr<T> result;

	if (index < m_colorTargetCount)
	{
		result = m_textures[TARGETTYPE_COLORBASE + index];
	}

	return result;
}

template <class T>
void OsgRenderTarget<T>::useDepthTarget(bool val)
{
	if (val)
	{
		setupTexture(TARGETTYPE_DEPTH);
	}
	else
	{
		m_textures[TARGETTYPE_DEPTH] = nullptr;
	}
}

template <class T>
bool OsgRenderTarget<T>::doesUseDepthTarget() const
{
	return m_textures.at(TARGETTYPE_DEPTH) != nullptr;
}

template <class T>
std::shared_ptr<Texture> OsgRenderTarget<T>::getDepthTarget() const
{
	return m_textures.at(TARGETTYPE_DEPTH);
}

template <class T>
std::shared_ptr<T> OsgRenderTarget<T>::getDepthTargetOsg() const
{
	return m_textures.at(TARGETTYPE_DEPTH);
}


template <class T>
void OsgRenderTarget<T>::useStencilTarget(bool val)
{
	if (val)
	{
		setupTexture(TARGETTYPE_STENCIL);
	}
	else
	{
		m_textures[TARGETTYPE_STENCIL] = nullptr;
	}
}

template <class T>
bool OsgRenderTarget<T>::doesUseStencilTarget() const
{
	return m_textures.at(TARGETTYPE_STENCIL) != nullptr;
}

template <class T>
std::shared_ptr<Texture> OsgRenderTarget<T>::getStencilTarget() const
{
	return m_textures.at(TARGETTYPE_STENCIL);
}

template <class T>
std::shared_ptr<T> OsgRenderTarget<T>::getStencilTargetOsg() const
{
	return m_textures.at(TARGETTYPE_STENCIL);
}

template <class T>
void OsgRenderTarget<T>::rebuildTextures()
{
	for (int i = 0; i < OsgSupportedTextureCount; ++i)
	{
		if (m_textures[i] != nullptr)
		{
			setupTexture(i);
		}
	}
}

template <class T>
void OsgRenderTarget<T>::setupTexture(int type)
{
	if (m_textures[type] == nullptr)
	{
		m_textures[type] = std::make_shared<T>();
		m_textures[type]->setSize(m_width*m_scale, m_height*m_scale);
		osg::Texture* osgTexture = m_textures[type]->getOsgTexture();
		// We are not dealing with mipmaps, fix up the filters to enable rendering to FBO
		// see http://www.opengl.org/wiki/Common_Mistakes#Creating_a_complete_texture
		osgTexture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
		osgTexture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
		osgTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE); 
		osgTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
		if (type == TARGETTYPE_DEPTH)
		{
			osgTexture->setSourceFormat(GL_DEPTH_COMPONENT); 
			osgTexture->setSourceType(GL_FLOAT); 
			osgTexture->setInternalFormat(GL_DEPTH_COMPONENT32F);
		}
		if (type >= TARGETTYPE_COLORBASE)
		{
			osgTexture->setInternalFormat(GL_RGBA32F_ARB); 
			osgTexture->setSourceFormat(GL_RGBA); 
			osgTexture->setSourceType(GL_FLOAT);
		}
	}
}

#endif