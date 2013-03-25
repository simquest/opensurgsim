#ifndef SURGSIM_GRAPHICS_OSG_MATERIAL_IMPLEMENTATION_H
#define SURGSIM_GRAPHICS_OSG_MATERIAL_IMPLEMENTATION_H

#include <SurgSim/Graphics/MaterialImplementation.h>

namespace SurgSim 
{
	namespace Graphics
	{
		namespace Osg
		{
			class MaterialImplementation : public SurgSim::Graphics::MaterialImplementation
			{
			public:
				MaterialImplementation() : SurgSim::Graphics::MaterialImplementation()
				{
				}

				virtual ~MaterialImplementation()
				{
				}
			};
		}
	}
}

#endif