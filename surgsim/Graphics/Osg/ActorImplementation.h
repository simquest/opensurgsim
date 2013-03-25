#ifndef SURGSIM_GRAPHICS_OSG_ACTOR_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_OSG_ACTOR_IMPLEMENTATION_H

#include <SurgSim/Graphics/ActorImplementation.h>

#include <osg/Node>

namespace SurgSim 
{
	namespace Graphics
	{
		namespace Osg
		{
			class ActorImplementation : public SurgSim::Graphics::ActorImplementation
			{
			public:
				ActorImplementation() : SurgSim::Graphics::ActorImplementation()
				{
				}

				virtual ~ActorImplementation()
				{
				}

				osg::ref_ptr<osg::Node> getNode() const
				{
					return m_node;
				}
			private:
				osg::ref_ptr<osg::Node> m_node;
			};
		}
	}
}

#endif