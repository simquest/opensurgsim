// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINTDATA_H
#define SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINTDATA_H

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/RigidRepresentationBase.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

/// Class for rotation vector constraint data between a rigid/fixed representation and Fem1d beam being controlled.
///
/// It considers the Fem1D rotational dof (beamRotationVector) to be the only variable to account for.
/// The equation to verify is \f$ R_{rigid} * RAtGrasp_{rigid}^{-1} = R_{beam} * RAtGrasp_{beam}^{-1} \f$
/// where \f$R\f$ denotes the current prefixed object 3x3 rotation matrix
///       \f$RAtGrasp\f$ is the 3x3 rotation matrix of the prefixed object at the time of the constraint creation
/// and \f$R_{beam}\f$ is decomposed into the initial rotation \f$R0_{beam}\f$ and the current rotation given by the
/// rotational dof
/// \f$R_{rigid} * RAtGrasp_{rigid}^{-1} = R(beamRotationVector) * R0_{beam} * RAtGrasp_{beam}^{-1}\f$
/// \f$R_{rigid} * RAtGrasp_{rigid}^{-1} * RAtGrasp_{beam} * R0_{beam}^{-1} = R(beamRotationVector)\f$
/// \f$rotationVector(R_{rigid} * RAtGrasp_{rigid}^{-1} * RAtGrasp_{beam} * R0_{beam}^{-1}) = beamRotationVector\f$
class RotationVectorRigidFem1DConstraintData : public ConstraintData
{
public:
	/// Default constructor
	RotationVectorRigidFem1DConstraintData() :
		ConstraintData()
	{
	}

	/// Destructor
	virtual ~RotationVectorRigidFem1DConstraintData()
	{
	}

	/// Set the rigid/fixed object part that will control the fem1d
	/// \param rigid The rigid base representation (either a RigidRepresentation or FixedRepresentation)
	/// \param rigidRAtGrasp The rigid rotation at the time of the constraint creation
	void setRigidOrFixedRotation(std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> rigid,
		const SurgSim::Math::Matrix33d& rigidRAtGrasp)
	{
		SURGSIM_ASSERT(nullptr != rigid) << "Need a valid rigid/fixed representation";

		m_rigidRAtGrasp = rigidRAtGrasp;
		m_rigid = rigid;
	}

	/// Set the fem1d object part
	/// \param beams The Fem1DRepresentation to be controlled by the rigid/fixed representation orientation
	/// \param beamId The beam id that is going to be controlled by the rigid/fixed representation orientation
	void setFem1DRotation(std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> beams, size_t beamId)
	{
		SURGSIM_ASSERT(nullptr != beams) << "Need a valid fem1D representation";
		SURGSIM_ASSERT(beams->getNumFemElements() > beamId) << "The beam id " << beamId
			<< " does not exists, the fem1d has " << beams->getNumFemElements() << " beams";

		auto beam = std::dynamic_pointer_cast<SurgSim::Physics::Fem1DElementBeam>(beams->getFemElement(beamId));

		m_beams = beams;

		m_beamR0 = beam->getInitialRotation();

		const auto& rotVecBeamNode0 = beams->getCurrentState()->getPositions().segment<3>(6 * beam->getNodeId(0) + 3);
		SurgSim::Math::Matrix33d R = SurgSim::Math::Matrix33d::Identity();
		if (!rotVecBeamNode0.isApprox(SurgSim::Math::Vector3d::Zero()))
		{
			R = SurgSim::Math::makeRotationMatrix(rotVecBeamNode0.norm(), rotVecBeamNode0.normalized());
		}
		m_beamRAtGrasp = R * m_beamR0;
	}

	/// \return The current rotation vector that should correspond to the beam rotation vector
	/// i.e. \f$ rotationVector(R_{rigid} * RAtGrasp_{rigid}^{-1} * RAtGrasp_{beam} * R0_{beam}^{-1}) \f$
	SurgSim::Math::Vector3d getCurrentRotationVector() const
	{
		SURGSIM_ASSERT(nullptr != m_rigid) << "Did you call setRigidOrFixedRotation prior to using this class ?";
		SURGSIM_ASSERT(nullptr != m_beams) << "Did you call setFem1DRotation prior to using this class ?";

		const auto& rigidR = m_rigid->getCurrentState().getPose().linear();
		Eigen::AngleAxisd angleAxis(rigidR * m_rigidRAtGrasp.inverse() * m_beamRAtGrasp * m_beamR0.inverse());
		return angleAxis.angle() * angleAxis.axis();
	}

private:
	/// Rigid/Fixed representation
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> m_rigid;
	/// Rigid/Fixed rotation information at the time of the constraint creation
	SurgSim::Math::Matrix33d m_rigidRAtGrasp;

	/// Fem1D representation
	std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> m_beams;
	/// The beam initial rotation and rotation at the time of the constraint creation
	SurgSim::Math::Matrix33d m_beamR0, m_beamRAtGrasp;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINTDATA_H
