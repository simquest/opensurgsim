// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/PostprocessingView.h"

#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "../Graphics/OsgProgram.h"
#include "../Framework/Runtime.h"

namespace SurgSim
{
namespace Blocks
{


PostprocessingView::PostprocessingView(
	const std::string& name,
	const std::string& shaderSource) :
	Framework::SceneElement(name),
	m_defaultCamera(std::make_shared<Graphics::OsgCamera>("DefaultCamera")),
	m_toScreenCamera(std::make_shared<Graphics::OsgCamera>("ToScreenCamera")),
	m_view(std::make_shared<Graphics::OsgView>("View"))
{

	auto dim = m_view->getDimensions();
	m_view->setTargetScreen(1);
	//m_view->setStereoMode(Graphics::View::STEREO_MODE_HORIZONTAL_SPLIT);
	m_view->enableManipulator(true);

	m_view->setCamera(m_defaultCamera);

	m_defaultCamera->setRenderGroupReference(Graphics::Representation::DefaultGroupName);
	m_defaultCamera->setRenderTarget(std::make_shared<Graphics::OsgRenderTarget2d>(dim[0], dim[1], 1.0, 1, true, true));
	m_defaultCamera->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_IN_ORDER, 0);
	m_defaultCamera->setMainCamera(true);
	m_defaultCamera->getOsgCamera()->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 1.0));
	m_defaultCamera->getOsgCamera()->setReferenceFrame(osg::Transform::RELATIVE_RF);

	// The default camera is the one that is the camera that carries the main information, the toScreenCamera is just
	// a copier from the postProcessed texture to the screen

	m_material = std::make_shared<Graphics::OsgMaterial>("TextureToScreenMaterial");

	m_material->addUniform("sampler2D", "renderTarget", m_defaultCamera->getRenderTarget()->getColorTarget(0));
	m_material->getUniform("renderTarget")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	
	// This is only for the default material
	m_material->addUniform("float", "gamma", 0.9f);
	m_material->addUniform("float", "exposure", 1.0f);

	m_material->loadProgram(shaderSource);
	m_material->getProgram()->setGlobalScope(true);

	auto vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(-1.0, -1.0, 0.0));
	vertices->push_back(osg::Vec3(1.0, -1.0, 0.0));
	vertices->push_back(osg::Vec3(1.0, 1.0, 0.0));
	vertices->push_back(osg::Vec3(-1.0, 1.0, 0.0));

	auto geom = new osg::Geometry;
	geom->setVertexArray(vertices);

	auto elements = new osg::DrawElementsUByte(GL_TRIANGLE_STRIP, 0);
	elements->push_back(0);
	elements->push_back(1);
	elements->push_back(3);
	elements->push_back(2);
	geom->addPrimitiveSet(elements);

	auto geode = new osg::Geode;
	geode->addDrawable(geom);
	geode->setName("The Quad that should be drawn");

	m_toScreenCamera->getOsgCamera()->getChild(0)->asGroup()->addChild(geode);

	m_toScreenCamera->setMaterial(m_material);
	m_toScreenCamera->setRenderOrder(SurgSim::Graphics::Camera::RENDER_ORDER_POST_RENDER, 1000);
	m_toScreenCamera->setMainCamera(false);
	//m_toScreenCamera->setRenderGroupReference("__OSS_NULL__");
	auto osgCamera = m_toScreenCamera->getOsgCamera();
	osgCamera->setCullingMode(osg::Camera::NO_CULLING);
	osgCamera->setClearMask(0x0);
	osgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
}

void PostprocessingView::enableManipulator(bool val)
{
	// m_view->enableManipulator(true);
}

std::shared_ptr<SurgSim::Graphics::Camera> PostprocessingView::getCamera()
{
	return m_defaultCamera;
}

std::shared_ptr<SurgSim::Graphics::Camera> PostprocessingView::getPostProcessingCamera()
{
	return m_toScreenCamera;
}

bool PostprocessingView::doInitialize()
{
	addComponent(m_view);
	addComponent(m_defaultCamera);
	addComponent(m_material);
	addComponent(m_toScreenCamera);

	return true;
}


std::shared_ptr<SurgSim::Graphics::Material> PostprocessingView::getPostProcessingMaterial()
{
	return m_material;
}

void PostprocessingView::setPostprocessingMaterial(std::shared_ptr<Graphics::OsgMaterial> material)
{
	if (m_material != nullptr) {
		removeComponent(m_material);
	}
	addComponent(material);
	m_material = material;
	m_toScreenCamera->setMaterial(m_material);
}

}
}
