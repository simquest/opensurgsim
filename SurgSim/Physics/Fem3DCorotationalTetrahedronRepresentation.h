// This file is a part of the SimQuest OpenSurgSim extension.
// Copyright 2012-2017, SimQuest Solutions Inc.

#ifndef PHYSICS_Fem3DCorotationalTetrahedronRepresentation_H
#define PHYSICS_Fem3DCorotationalTetrahedronRepresentation_H

#include <memory>
#include <string>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

namespace SurgSim
{

	namespace Physics
	{
		SURGSIM_STATIC_REGISTRATION(Fem3DCorotationalTetrahedronRepresentation);

		class FemPlyReaderDelegate;

		/// Co-rotational Finite Element Model 3D is a fem built with co-rotational 3D FemElement
		/// It derives from Fem1DRepresentation from which it uses most functionalities.
		/// The only difference comes in the initialization and update to take a special care of
		/// the rotational dof.
		class Fem3DCorotationalTetrahedronRepresentation : public SurgSim::Physics::Fem3DRepresentation
		{
		public:
			/// Constructor
			/// \param name The name of the Fem3DCorotationalTetrahedronRepresentation
			explicit Fem3DCorotationalTetrahedronRepresentation(const std::string& name);

			/// Destructor
			virtual ~Fem3DCorotationalTetrahedronRepresentation();

			SURGSIM_CLASSNAME(SimQuest::Physics::Fem3DCorotationalTetrahedronRepresentation);

			void setFemElementType(const std::string& type) override;

		protected:
			SurgSim::Math::Matrix getNodeTransformation(const SurgSim::Math::OdeState& state, size_t nodeId) override;
		};

	} // namespace Physics

} // namespace SimQuest

#endif // PHYSICS_Fem3DCorotationalTetrahedronRepresentation_H
