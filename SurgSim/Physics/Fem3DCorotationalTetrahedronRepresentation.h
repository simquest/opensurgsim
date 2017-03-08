// This file is a part of the SimQuest OpenSurgSim extension.
// Copyright 2012-2017, SimQuest Solutions Inc.

#ifndef PHYSICS_TETRAHEDRALCOROTATIONALFEM3DREPRESENTATION_H
#define PHYSICS_TETRAHEDRALCOROTATIONALFEM3DREPRESENTATION_H

#include <memory>
#include <string>

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"

namespace SimQuest
{

	namespace Physics
	{
		SURGSIM_STATIC_REGISTRATION(TetrahedralCorotationalFem3DRepresentation);

		class FemPlyReaderDelegate;

		/// Co-rotational Finite Element Model 3D is a fem built with co-rotational 3D FemElement
		/// It derives from Fem1DRepresentation from which it uses most functionalities.
		/// The only difference comes in the initialization and update to take a special care of
		/// the rotational dof.
		class TetrahedralCorotationalFem3DRepresentation : public SurgSim::Physics::Fem3DRepresentation
		{
		public:
			/// Constructor
			/// \param name The name of the TetrahedralCorotationalFem3DRepresentation
			explicit TetrahedralCorotationalFem3DRepresentation(const std::string& name);

			/// Destructor
			virtual ~TetrahedralCorotationalFem3DRepresentation();

			SURGSIM_CLASSNAME(SimQuest::Physics::TetrahedralCorotationalFem3DRepresentation);

			void setFemElementType(const std::string& type) override;

		protected:
			SurgSim::Math::Matrix getNodeTransformation(const SurgSim::Math::OdeState& state, size_t nodeId) override;
		};

	} // namespace Physics

} // namespace SimQuest

#endif // PHYSICS_TETRAHEDRALCOROTATIONALFEM3DREPRESENTATION_H
