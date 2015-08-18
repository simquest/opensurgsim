#include "ImplicitFluidRendering.h"

#include "SurgSim/Framework/PoseComponent.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRenderTarget.h"
#include "SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h"
#include "SurgSim/Graphics/OsgTexture2d.h"
#include "SurgSim/Graphics/OsgTextureUniform.h"
#include "SurgSim/Graphics/RenderPass.h"
#include "SurgSim/Graphics/View.h"

namespace SurgSim
{
namespace Blocks
{

std::shared_ptr<Framework::SceneElement> createShadeingQuad(
	std::shared_ptr<Framework::TransferPropertiesBehavior> copier,
	std::shared_ptr<Graphics::Camera> camera,
	std::shared_ptr<Graphics::Texture> depthMap,
	std::shared_ptr<Graphics::Texture> normalMap,
	int width, int height)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("ImplicitSurfaceShading");

	auto material = Graphics::buildMaterial("Shaders/pointsplat/surface.vert", "Shaders/pointsplat/surface.frag");
	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthMap);
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("sampler2D", "normalMap");
	material->setValue("normalMap", normalMap);
	material->getUniform("normalMap")->setValue("MinimumTextureUnit", static_cast<size_t>(9));
	material->addUniform("vec4", "color");
	material->setValue("color", Math::Vector4f(0.3, 0.0, 0.05, 1.0));
	material->addUniform("mat4", "inverseProjectionMatrix");

	/*
	auto renderCamera = std::make_shared<Graphics::OsgCamera>("Camera");
	renderCamera->setViewport(0, width, 0, height);
	renderCamera->setLocalPose(SurgSim::Math::makeRigidTransform(
								   Math::Vector3d(0, 0, -1.0),
								   Math::Vector3d(0, 0, 0),
								   Math::Vector3d(0.0, 1.0, 0.0)));
	renderCamera->getOsgCamera()->setProjectionMatrixAsOrtho2D(0, width, 0, height);
	renderCamera->getOsgCamera()->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	renderCamera->getOsgCamera()->setClearColor(osg::Vec4f(0.8, 0.0, 0.0, 1.0));
	renderCamera->getOsgCamera()->setClearMask(0x0);
	renderCamera->setRenderOrder(Graphics::Camera::RENDER_ORDER_POST_RENDER, 0);
	renderCamera->setRenderGroupReference("ImplicitSurfaceShading");
	renderCamera->setGroupReference(Graphics::Camera::DefaultGroupName);
	element->addComponent(renderCamera);
	*/
	copier->connect(camera, "FloatInverseProjectionMatrix", material, "inverseProjectionMatrix");
	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Graphics");
	graphics->setSize(width , height);
	graphics->setLocation(0, 0);
	graphics->setMaterial(material);
	// graphics->setGroupReference("ImplicitSurfaceShading");
	element->addComponent(graphics);
	element->addComponent(material);
	return element;
}

void creatFluidRenderingPass(const float& sphereRadius, std::shared_ptr<Graphics::OsgViewElement> viewElement,
							 std::shared_ptr<Framework::Scene> scene)
{
	auto copier =  std::make_shared<Framework::TransferPropertiesBehavior>("Copier");
	copier->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);
	viewElement->addComponent(copier);

	std::array<int, 2> dimensions = viewElement->getView()->getDimensions();
	int screenWidth = dimensions[0];
	int screenHeight = dimensions[1];

	// Depth Pass //
	auto depthPass = std::make_shared<Graphics::RenderPass>("DepthPass");
	depthPass->getCamera()->setRenderGroupReference("DepthPass");
	depthPass->getCamera()->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 0);

	copier->connect(viewElement->getPoseComponent(), "Pose", depthPass->getCamera(), "LocalPose");
	copier->connect(viewElement->getCamera(), "ProjectionMatrix", depthPass->getCamera() , "ProjectionMatrix");

	auto renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 0, true);
	depthPass->setRenderTarget(renderTarget);

	auto material = Graphics::buildMaterial("Shaders/pointsplat/sphere_depth.vert", "Shaders/pointsplat/sphere_depth.frag");

	auto texture = std::make_shared<Graphics::OsgTexture2d>();
	texture->setIsPointSprite(true);
	auto pointSpriteUniform = std::make_shared<Graphics::OsgTextureUniform<Graphics::OsgTexture2d>>("psDepth");
	pointSpriteUniform->set(texture);
	material->addUniform(pointSpriteUniform);

	material->addUniform("float", "sphereRadius");
	material->setValue("sphereRadius", sphereRadius);
	material->addUniform("float", "sphereScale");
	material->setValue("sphereScale", 800.0f);
	depthPass->setMaterial(material);
	depthPass->showDepthTarget(0, 0, 256, 256);

	scene->addSceneElement(depthPass);


	// Normal Pass //
	auto normalPass = std::make_shared<Graphics::RenderPass>("NormalPass");
	normalPass->getCamera()->setRenderGroupReference("NormalPass");
	normalPass->getCamera()->setGroupReference(Graphics::Representation::DefaultGroupName);

	auto camera = std::dynamic_pointer_cast<Graphics::OsgCamera>(normalPass->getCamera());
	auto osgCamera = camera->getOsgCamera();
	osgCamera->setViewport(0, 0, 1024, 1024);
	camera->setOrthogonalProjection(0, 1024, 0, 1024, -1.0, 1.0);

	camera->setRenderOrder(Graphics::Camera::RENDER_ORDER_PRE_RENDER, 1);

	renderTarget = std::make_shared<Graphics::OsgRenderTarget2d>(1024, 1024, 1.0, 1, false);
	normalPass->setRenderTarget(renderTarget);

	material = Graphics::buildMaterial("Shaders/pointsplat/sphere_normal.vert", "Shaders/pointsplat/sphere_normal.frag");

	material->addUniform("sampler2D", "depthMap");
	material->setValue("depthMap", depthPass->getRenderTarget()->getDepthTarget());
	material->getUniform("depthMap")->setValue("MinimumTextureUnit", static_cast<size_t>(8));
	material->addUniform("float", "texelSize");
	material->setValue("texelSize", static_cast<float>(1.0 / 1024.0));
	material->addUniform("mat4", "inverseProjectionMatrix");

	copier->connect(viewElement->getCamera(), "FloatInverseProjectionMatrix", material, "inverseProjectionMatrix");

	normalPass->setMaterial(material);
	normalPass->showColorTarget(256, 0, 256, 256);

	// Quad
	auto graphics = std::make_shared<Graphics::OsgScreenSpaceQuadRepresentation>("Quad");
	graphics->setSize(1024, 1024);
	graphics->setLocation(0, 0);
	graphics->setGroupReference("NormalPass");
	normalPass->addComponent(graphics);

	scene->addSceneElement(normalPass);


	// Shading Pass //

	auto shader = createShadeingQuad(copier, viewElement->getCamera(),
									 depthPass->getRenderTarget()->getDepthTarget(),
									 normalPass->getRenderTarget()->getColorTarget(0),
									 screenWidth, screenHeight);

	scene->addSceneElement(shader);
}
} // Blocks
} // SurgSim
