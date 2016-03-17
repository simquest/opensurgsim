#include "SurgSim/Graphics/DecalBehavior.h"


namespace SurgSim
{
namespace Graphics
{
DecalBehavior::DecalBehavior(const std::string& name) : 
	Framework::Behavior(name)
{
}

void DecalBehavior::setPainter(std::shared_ptr<Collision::Representation> painter)
{
	m_painter = painter;
}

std::shared_ptr<Collision::Representation> DecalBehavior::getPainter() const
{
	return m_painter;
}

void DecalBehavior::setTexture(std::shared_ptr<Graphics::Texture> texture)
{
	m_texture = texture;
}

std::shared_ptr<Graphics::Texture> DecalBehavior::getTexture() const
{
	return m_texture;
}

void DecalBehavior::setDecalColor(Math::Vector4f color)
{
	m_color = color;
}

Math::Vector4f DecalBehavior::getDecalColor() const
{
	return m_color;
}

	bool DecalBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_painter != nullptr) << "You must provide a collision representation that will paint the decals.";
	SURGSIM_ASSERT(m_texture != nullptr) << "You must provide a texture to paint decals onto.";
	
	// Check that the image data of the texture are valid?
	//SURGSIM_ASSERT(m_texture.getWidth() > 0 && m_texture.getHeight() > 0 && m_texture.getNumChannels() == 4)
	//	<< "You must provide a texture image that provides RGBA channels.";

	return true;
}

bool DecalBehavior::doWakeUp()
{
	// Something goes here
	return true;
}

void DecalBehavior::update(double dt)
{
	// Filter collisions to m_painter
	// In AR the behavior to grip the agent receives a ContactMapType of collisions and goes through them
	// Is there something built into OSS to do filtering?

	// Calculate contact point to nearest graphics representation vertex via getSceneElement()->getComponent("Graphics")->getMesh()
	// Some built in functions were mentioned

	// Get texture coordinate of vertex with mesh->getVertex(id).data.texture
	size_t s = 0.0;
	size_t t = 0.0;

	// Paint a shape of some kind on the image (allow use of a texture for a decal?)

	// Need to implement a generic method to get the image data from the texture as an Image class
	//auto data = m_texture->getImage()(s, t);
	// Getting data from Image class returns an Eigen::Map, need to look at how to alter data within that and apply m_color

	// Should this be additive to what already exists to the image data or overwrite?

	// Need to implemenet generic dirty method since OSG needs this
	//m_texture->dirty();
}

} // Graphics
} // SurgSim