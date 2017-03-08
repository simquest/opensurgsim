// This file is a part of the SimQuest OpenSurgSim extension.
// Copyright 2012-2017, SimQuest Solutions Inc.

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/LinearSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/FemConstraintFixedPoint.h"
#include "SurgSim/Physics/FemConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FemConstraintFrictionalSliding.h"
#include "SurgSim/Physics/FemConstraintFrictionlessContact.h"
#include "SurgSim/Physics/FemConstraintFrictionlessSliding.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"

#include "SimQuest/Math/RotationVector.h"
#include "SimQuest/Physics/TetrahedralCorotationalFem3DRepresentation.h"

namespace SimQuest
{

	namespace Physics
	{

		SURGSIM_REGISTER(SurgSim::Framework::Component,
			SimQuest::Physics::TetrahedralCorotationalFem3DRepresentation,
			TetrahedralCorotationalFem3DRepresentation);

		TetrahedralCorotationalFem3DRepresentation::TetrahedralCorotationalFem3DRepresentation(const std::string& name)
			: Fem3DRepresentation(name)
		{
			using SurgSim::Physics::ConstraintImplementation;
			using SurgSim::Physics::FemConstraintFixedPoint;
			using SurgSim::Physics::FemConstraintFixedRotationVector;
			using SurgSim::Physics::FemConstraintFrictionalSliding;
			using SurgSim::Physics::FemConstraintFrictionlessContact;
			using SurgSim::Physics::FemConstraintFrictionlessSliding;

			SurgSim::Physics::Fem3DElementCorotationalTetrahedron tetCoro;
			setFemElementType(tetCoro.getClassName());

			setComplianceWarping(true);

			// Register all the constraint for this representation in the ConstraintImplementation factory
			// Because TetrahedralCorotationalFem3DRepresentation derives from Fem1DRepresentation, it can use the exact
			// same constraint implementation. The constraint expression is exactly the same and the compliance
			// used will be the correct one.
			ConstraintImplementation::getFactory().addImplementation(
				typeid(TetrahedralCorotationalFem3DRepresentation), std::make_shared<FemConstraintFixedPoint>());
			ConstraintImplementation::getFactory().addImplementation(
				typeid(TetrahedralCorotationalFem3DRepresentation), std::make_shared<FemConstraintFixedRotationVector>());
			ConstraintImplementation::getFactory().addImplementation(
				typeid(TetrahedralCorotationalFem3DRepresentation), std::make_shared<FemConstraintFrictionlessContact>());
			ConstraintImplementation::getFactory().addImplementation(
				typeid(TetrahedralCorotationalFem3DRepresentation), std::make_shared<FemConstraintFrictionlessSliding>());
			ConstraintImplementation::getFactory().addImplementation(
				typeid(TetrahedralCorotationalFem3DRepresentation), std::make_shared<FemConstraintFrictionalSliding>());
		}

		TetrahedralCorotationalFem3DRepresentation::~TetrahedralCorotationalFem3DRepresentation()
		{
		}

		void TetrahedralCorotationalFem3DRepresentation::setFemElementType(const std::string& type)
		{
			SurgSim::Physics::Fem3DElementCorotationalTetrahedron tetCoro;
			SURGSIM_ASSERT(type == tetCoro.getClassName()) <<
				"Invalid FemElement type found '" << type << "', default and expected is '" << tetCoro.getClassName() << "'";
			Fem3DRepresentation::setFemElementType(type);
		}

		SurgSim::Math::Matrix TetrahedralCorotationalFem3DRepresentation::getNodeTransformation(
			const SurgSim::Math::OdeState& state, size_t nodeId)
		{
			return SurgSim::Math::Matrix33d::Identity();

			using SimQuest::Math::rotationVectorToMatrix;

			std::vector<SurgSim::Math::Matrix33d> elementRotations;
			SurgSim::Math::Matrix33d R3x3;

			for (auto const &element : m_femElements)
			{
				auto node = std::find(element->getNodeIds().begin(), element->getNodeIds().end(), nodeId);
				if (node != element->getNodeIds().end())
				{
					elementRotations.push_back(std::static_pointer_cast<SurgSim::Physics::Fem3DElementCorotationalTetrahedron>(element)->m_R);
				}
			}

			if (elementRotations.size())
			{
				R3x3 = elementRotations[0];
			}
			else
			{
				R3x3 = elementRotations[0];
				for (size_t i = 1; i < elementRotations.size(); i++)
				{
					R3x3 = Eigen::Quaterniond(R3x3).slerp(0.5, Eigen::Quaterniond(elementRotations[i]));
				}
			}

			SurgSim::Math::Matrix rotation = SurgSim::Math::Matrix::Zero(getNumDofPerNode(), getNumDofPerNode());
			rotation = R3x3;

			return rotation;
		}

	} // namespace Physics

} // namespace SimQuest
