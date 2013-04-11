#ifndef SURGSIM_GRAPHICS_MATERIAL_H
#define SURGSIM_GRAPHICS_MATERIAL_H

#include <memory>

#include "MaterialImplementation.h"

namespace SurgSim 
{
	namespace Graphics
	{
		class Material
		{
		public:
			Material(std::shared_ptr<MaterialImplementation> implementation) : m_implementation(implementation)
			{
			}

			std::shared_ptr<MaterialImplementation> getImplementation()
			{
				return m_implementation;
			}

		private:
			std::shared_ptr<MaterialImplementation> m_implementation;
		};
	}
}

#endif