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

#ifndef SURGSIM_GRAPHICS_OSGRENDERTARGET_H
#define SURGSIM_GRAPHICS_OSGRENDERTARGET_H

#include <memory>
#include <unordered_map>

#include <SurgSim/Graphics/RenderTarget.h>
#include <SurgSim/Graphics/OsgTexture2d.h>
#include <SurgSim/Graphics/OsgTextureRectangle.h>

namespace SurgSim
{
namespace Graphics
{

template <class T>
class OsgRenderTarget : public RenderTarget
{
public:

	typedef T TextureType;

	/// Constructor
	OsgRenderTarget();
	OsgRenderTarget(int width, int height, double scale = 1.0,
					int colorCount = 0, bool useDepth = false, bool useStencil = false);
	~OsgRenderTarget();

	virtual void setSize(int width, int height) override;
	virtual void getSize(int* width, int* height) const override;

	virtual void setScale(double scale) override;
	virtual double getScale() const override;

	virtual int setColorTargetCount(int count) override;
	virtual int getColorTargetCount() const override;
	virtual std::shared_ptr<Texture> getColorTarget(int index) const override;
	virtual std::shared_ptr<T> getColorTargetOsg(int index) const;

	virtual void useDepthTarget(bool val) override;
	virtual bool doesUseDepthTarget() const override;
	virtual std::shared_ptr<Texture> getDepthTarget() const override;
	virtual std::shared_ptr<T> getDepthTargetOsg() const;

	virtual void useStencilTarget(bool val) override;
	virtual bool doesUseStencilTarget() const override;
	virtual std::shared_ptr<Texture> getStencilTarget() const override;
	virtual std::shared_ptr<T> getStencilTargetOsg() const;

private:

	enum TargetTypes {
		TARGETTYPE_DEPTH = 0,
		TARGETTYPE_STENCIL = 1,
		TARGETTYPE_COLORBASE = 2
	};

	int m_width;
	int m_height;

	double m_scale;

	int m_colorTargetCount;

	std::vector<std::shared_ptr<T>> m_textures;


	void rebuildTextures();
	void setupTexture(int type);
};

#include "OsgRenderTarget-inl.h"

typedef OsgRenderTarget<OsgTexture2d> OsgRenderTarget2d;
typedef OsgRenderTarget<OsgTextureRectangle> OsgRenderTargetRectangle;

}; // Graphics
}; // SurgSim

#endif